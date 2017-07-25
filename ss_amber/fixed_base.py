#!/usr/bin/env python2

from time import sleep

import numpy as np
from geometry.objects2 import World, Universe, WorldState

from autil.globals import glob
from robot.rrt import interpolatePath
from robot.pr2.pr2Robot import makeRobot
import graphics.windowManager3D as wm
from stripstream.algorithms.incremental.incremental_planner import incremental_planner
from stripstream.algorithms.search.fast_downward import get_fast_downward
from stripstream.pddl.utils import convert_plan
from stripstream.pddl.objects import EasyType, EasyParameter
from stripstream.pddl.logic.predicates import EasyPredicate
from stripstream.pddl.operators import Action, Axiom
from stripstream.pddl.logic.quantifiers import ForAll, Exists
from stripstream.pddl.logic.connectives import Not, And, Or
from stripstream.pddl.logic.atoms import Equal
from stripstream.pddl.utils import rename_easy
from stripstream.pddl.problem import STRIPStreamProblem
from stripstream.pddl.cond_streams import MultiEasyGenStream, EasyTestStream
from stripstream.utils import SEPARATOR

from ss_amber.problems import dantam2
from ss_amber.box_bodies import get_box_top_grasps
from ss_amber.utils import get_window_name, is_window_active, inverse_kinematics, grab, release, arm_motion, Traj, \
  set_pose, manip_from_pose_grasp, approach_from_manip

glob.usePR2(big_workspace=True)

# TODO - extract out useful files and ignore the rest (kind of like what I did with FFRob)

####################

ARM = 'left'
MAX_GRASPS = 1
DISABLE_TRAJECTORIES = False
DISABLE_TRAJ_COLLISIONS = True

print SEPARATOR
assert not DISABLE_TRAJECTORIES or DISABLE_TRAJ_COLLISIONS
if DISABLE_TRAJECTORIES:
  print 'Warning: trajectories are disabled'
if DISABLE_TRAJ_COLLISIONS:
  print 'Warning: trajectory collisions are disabled'

####################

Type, Pred, Param = EasyType, EasyPredicate, EasyParameter

# Types
OBJ, POSE, GRASP = Type(), Type(), Type()
CONF, TRAJ = Type(), Type()

# Fluents
ConfEq = Pred(CONF)
PoseEq = Pred(OBJ, POSE)
GraspEq = Pred(OBJ, GRASP)
HandEmpty = Pred()

# Derived
SafePose = Pred(OBJ, POSE)
SafeTraj = Pred(OBJ, TRAJ)

# Static trajectory
FreeMotion = Pred(CONF, CONF, TRAJ)
HoldingMotion = Pred(CONF, CONF, GRASP, TRAJ)
GraspMotion = Pred(POSE, GRASP, CONF, TRAJ)

# Static collision
CFreePose = Pred(POSE, POSE)
CFreeTraj = Pred(TRAJ, POSE)

O, P, G = Param(OBJ), Param(POSE), Param(GRASP)
O2, P2 = Param(OBJ), Param(POSE)
Q, Q2 = Param(CONF), Param(CONF)
T = Param(TRAJ)

rename_easy(locals())

####################

actions = [
  Action(name='pick', parameters=[O, P, G, Q, T],
    condition=And(PoseEq(O, P), HandEmpty(), ConfEq(Q), GraspMotion(P, G, Q, T),
                  ForAll([O2], Or(Equal(O, O2), SafeTraj(O2, T)))),
                  #ForAll([O2], Or(Equal(O, O2), And(SafePose(O2, P), SafeGTraj(O2, GT))))),
    effect=And(GraspEq(O, G), Not(HandEmpty()), Not(PoseEq(O, P)))),

  Action(name='place', parameters=[O, P, G, Q, T],
    condition=And(GraspEq(O, G), ConfEq(Q), GraspMotion(P, G, Q, T),
                  ForAll([O2], Or(Equal(O, O2), And(SafePose(O2, P), SafeTraj(O2, T))))),
    effect=And(PoseEq(O, P), HandEmpty(), Not(GraspEq(O, G)))),

  Action(name='move', parameters=[Q, Q2, T],
    condition=And(ConfEq(Q), HandEmpty(), FreeMotion(Q, Q2, T),
                  ForAll([O2], SafeTraj(O2, T))),
    effect=And(ConfEq(Q2), Not(ConfEq(Q)))),

  Action(name='move_holding', parameters=[Q, Q2, T, O, G],
    condition=And(ConfEq(Q), GraspEq(O, G), HoldingMotion(Q, Q2, G, T),
                  ForAll([O2], Or(Equal(O, O2), SafeTraj(O2, T)))),
    effect=And(ConfEq(Q2), Not(ConfEq(Q)))),
]

axioms = [
  Axiom(effect=SafePose(O2, P), condition=Exists([P2], And(PoseEq(O2, P2), CFreePose(P, P2)))),
  Axiom(effect=SafeTraj(O2, T), condition=Exists([P2], And(PoseEq(O2, P2), CFreeTraj(T, P2)))),
]

####################

# TODO - a version of this that uses as little as the world type things

def simple_test(use_window=True):
  universe = Universe()
  world = World(universe)
  robot = makeRobot(np.array(glob.workspace), useLeft=(ARM=='left'), useRight=(ARM=='right'))
  world.setRobot(robot)
  world_state = WorldState(world, robot) # realWorld
  window_3d = wm.makeWindow('World', glob.viewPort, 500, noWindow=(not use_window))

  problem = dantam2(world_state)
  if is_window_active(window_3d):
    window_3d.clear()
    world_state.draw(get_window_name(window_3d))
    raw_input('Continue?')

  bodies = {obj: world_state.objectShapes[obj] for obj in problem.object_names}
  all_bodies = bodies.values()
  body1 = all_bodies[-1] # Generic object 1
  body2 = all_bodies[-2] if len(bodies) >= 2 else body1 # Generic object 2
  grasps = get_box_top_grasps(body1)[:MAX_GRASPS]
  poses = problem.known_poses if problem.known_poses else []
  initial_conf = world_state.robotConf

  ####################

  # NOTE - can either modify the world state or create new bodies
  def cfree_pose(pose1, pose2): # Collision free test between an object at pose1 and an object at pose2
    return not body1.applyTrans(pose1).collides(body2.applyTrans(pose2))

  def _cfree_traj_pose(traj, pose): # Collision free test between a robot executing traj and an object at pose
    _enable_all(False)
    body2.Enable(True)
    set_pose(body2, pose.value)
    for conf in traj.path():
      set_manipulator_conf(manipulator, conf)
      if env.CheckCollision(robot, body2):
        return False
    return True

  def _cfree_traj_grasp_pose(traj, grasp, pose): # Collision free test between an object held at grasp while executing traj and an object at pose
    _enable_all(False)
    body1.Enable(True)
    body2.Enable(True)
    set_pose(body2, pose.value)
    for conf in traj.path():
      set_manipulator_conf(manipulator, conf)
      manip_trans = manipulator.GetTransform()
      set_pose(body1, object_trans_from_manip_trans(manip_trans, grasp.grasp_trans))
      if env.CheckCollision(body1, body2):
        wm.getWindow('W').clear(); env.realWorld.draw('W')
        raw_input('Collision in _cfree_traj_grasp_pose')
        return False
    return True

  def cfree_traj(traj, pose): # Collision free test between a robot executing traj (which may or may not involve a grasp) and an object at pose
    if DISABLE_TRAJ_COLLISIONS:
      return True
    return _cfree_traj_pose(traj, pose) and (traj.grasp is None or _cfree_traj_grasp_pose(traj, traj.grasp, pose))

  ####################

  def sample_grasp_traj(pose, grasp): # Sample pregrasp config and motion plan that performs a grasp
    manip_frame = manip_from_pose_grasp(pose, grasp, robot, ARM)
    grasp_conf = next(inverse_kinematics(initial_conf, manip_frame, ARM), None) # Grasp configuration
    if grasp_conf is None:
      return
    pregrasp_frame = approach_from_manip(manip_frame) # NOTE - I could also do this in the opposite order
    pregrasp_conf = next(inverse_kinematics(grasp_conf, pregrasp_frame, ARM), None) # Approach configuration
    if pregrasp_conf is None:
      return
    if DISABLE_TRAJECTORIES:
      yield [(pregrasp_conf, Traj([grasp_conf], grasp))]
      return
    grasp_path = interpolatePath([grasp_conf, pregrasp_conf]) # Trajectory from grasp configuration to pregrasp
    if grasp_path is None:
      return
    yield [(pregrasp_conf, Traj(grasp_path, grasp))]

  def sample_free_motion(conf1, conf2): # Sample motion while not holding
    if DISABLE_TRAJECTORIES:
      yield [(Traj([conf2]),)]
      return
    #_enable_all(False)
    path = arm_motion(world_state, conf1, conf2, ARM)
    if path is None:
      return
    yield [(Traj(path),)]

  def sample_holding_motion(conf1, conf2, grasp): # Sample motion while holding
    if DISABLE_TRAJECTORIES:
      yield [(Traj([conf2], grasp),)]
      return
    """
    previous_pose = world_state.getObjectPose(get_name(body1))
    manip_tform = forward_kinematics(conf1, ARM)
    pose = pose_from_manip_grasp(manip_tform, grasp, robot, ARM)
    set_pose(world_state, body1, pose)
    #draw(world_state, window_3d)
    #raw_input('awefawfe')

    Grab(world_state, body1, ARM)
    path = arm_motion(world_state, conf1, conf2)
    #Release(world_state, ARM) # Exception: Not a valid 2.5D Pose
    world_state.setObjectPose(get_name(world_state.detach(ARM)), previous_pose)
    """
    path = arm_motion(world_state, conf1, conf2, ARM)
    if path is None:
      return
    yield [(Traj(path, grasp),)]

  ####################

  cond_streams = [
    # Pick/place trajectory
    MultiEasyGenStream(inputs=[P, G], outputs=[Q, T], conditions=[],
                       effects=[GraspMotion(P, G, Q, T)], generator=sample_grasp_traj),

    # Move trajectory
    MultiEasyGenStream(inputs=[Q, Q2], outputs=[T], conditions=[],
                effects=[FreeMotion(Q, Q2, T)], generator=sample_free_motion, order=1, max_level=0),
    MultiEasyGenStream(inputs=[Q, Q2, G], outputs=[T], conditions=[],
                effects=[HoldingMotion(Q, Q2, G, T)], generator=sample_holding_motion, order=1, max_level=0),

    # Collisions
    EasyTestStream(inputs=[P, P2], conditions=[], effects=[CFreePose(P, P2)],
                test=cfree_pose, eager=True),
    EasyTestStream(inputs=[T, P], conditions=[], effects=[CFreeTraj(T, P)],
                test=cfree_traj),
  ]

  ####################

  constants = map(GRASP, grasps) + map(POSE, poses)
  initial_atoms = [
    ConfEq(initial_conf),
    HandEmpty(),
  ] + [
    PoseEq(obj, pose) for obj, pose in problem.initial_poses.iteritems()
  ]
  goal_formula = And(ConfEq(initial_conf), *(PoseEq(obj, pose) for obj, pose in problem.goal_poses.iteritems()))
  stream_problem = STRIPStreamProblem(initial_atoms, goal_formula, actions + axioms, cond_streams, constants)

  search_fn = get_fast_downward('eager', max_time=10, verbose=False)
  plan, _ = incremental_planner(stream_problem, search=search_fn, frequency=1, waves=True, debug=False)

  print SEPARATOR

  plan = convert_plan(plan)
  if plan is not None:
    print 'Success'
    for i, (action, args) in enumerate(plan):
      print i+1, action
  else:
    print 'Failure'

  if is_window_active(window_3d) and plan is not None:
    def _execute_traj(path):
      for j, conf in enumerate(path):
        world_state.setRobotConf(conf)
        window_3d.clear()
        world_state.draw(get_window_name(window_3d))
        #raw_input('%s/%s) Step?'%(j, len(path)))
        window_3d.update(); sleep(0.01)

    # Resets the initial state
    world_state.setRobotConf(initial_conf)
    for obj, pose in problem.initial_poses.iteritems():
      set_pose(world_state, bodies[obj], pose)

    window_3d.clear()
    world_state.draw(get_window_name(window_3d))
    for i, (action, args) in enumerate(plan):
      raw_input('\n%s/%s) Next?'%(i, len(plan)))
      if action.name == 'move':
        _, _, traj = args
        _execute_traj(traj.path)
      elif action.name == 'move_holding':
        _, _, traj, _, _ = args
        _execute_traj(traj.path)
      elif action.name == 'pick':
        obj, _, _, _, traj = args
        _execute_traj(traj.path[::-1])
        grab(world_state, bodies[obj], ARM)
        _execute_traj(traj.path)
      elif action.name == 'place':
        obj, _, _, _, traj = args
        _execute_traj(traj.path[::-1])
        release(world_state, ARM)
        _execute_traj(traj.path)
      else:
        raise ValueError(action.name)
      window_3d.clear()
      world_state.draw(get_window_name(window_3d))

  raw_input('Finish?')

##################################################

def main():
  simple_test()

if __name__ == '__main__':
  main()
  
