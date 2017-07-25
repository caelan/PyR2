#!/usr/bin/env python
from time import sleep

# bhpn interface (StripStreamSupport => sss)
from ss_tomas import sss

reload(sss)
from ss_tomas.sss import *

####################

PROBLEM = dantam2 # dantam | dantam2
ARM = 'leftarm'
MAX_GRASPS = 6
# USE_GRASP_APPROACH = GRASP_APPROACHES.TOP # TOP | SIDE
# USE_GRASP_TYPE = GRASP_TYPES.GRASP # GRASP | TOUCH
USE_GRASP_APPROACH = True
USE_GRASP_TYPE = None
DISABLE_TRAJECTORIES = False
DISABLE_TRAJ_COLLISIONS = False

assert not DISABLE_TRAJECTORIES or DISABLE_TRAJ_COLLISIONS
if DISABLE_TRAJECTORIES:
  print 'Warning: trajectories are disabled'
if DISABLE_TRAJ_COLLISIONS:
  print 'Warning: trajectory collisions are disabled'

####################

def simple_test(env):
  viewer = env.GetViewer() is not None
  problem = PROBLEM(env)

  robot = env.GetRobots()[0]
  print 'robot', robot
  set_base_conf(robot, (-.75, .2, -math.pi/2))
  initialize_openrave(env, ARM, min_delta=.01)
  manipulator = robot.GetActiveManipulator()
  cspace = CSpace.robot_arm(manipulator)
  base_manip = interfaces.BaseManipulation(robot, plannername=None, maxvelmult=None)

  bodies = {obj: env.GetKinBody(obj) for obj in problem.object_names}
  all_bodies = bodies.values()
  assert len({body.GetKinematicsGeometryHash() for body in all_bodies}) == 1 # NOTE - assuming all objects has the same geometry
  body1 = all_bodies[-1] # Generic object 1
  body2 = all_bodies[-2] if len(bodies) >= 2 else body1 # Generic object 2
  grasps = get_grasps(env, robot, body1, USE_GRASP_APPROACH, USE_GRASP_TYPE)[:MAX_GRASPS]
  poses = problem.known_poses if problem.known_poses else []

  open_gripper(manipulator)
  initial_conf = Conf(robot.GetConfigurationValues()[manipulator.GetArmIndices()])

  def _enable_all(enable): # Enables or disables all bodies for collision checking
    for body in all_bodies:
      body.Enable(enable)

  ####################

  def cfree_pose(pose1, pose2): # Collision free test between an object at pose1 and an object at pose2
    body1.Enable(True)
    set_pose(body1, pose1.value)
    body2.Enable(True)
    set_pose(body2, pose2.value)
    return not env.CheckCollision(body1, body2)

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
    _enable_all(False)
    body1.Enable(True)
    set_pose(body1, pose.value)
    manip_trans, approach_vector = manip_from_pose_grasp(pose, grasp)
    grasp_conf = solve_inverse_kinematics(env, manipulator, manip_trans) # Grasp configuration
    if grasp_conf is None: return
    if DISABLE_TRAJECTORIES:
      yield [(Conf(grasp_conf), object())]
      return

    set_manipulator_conf(manipulator, grasp_conf)
    robot.Grab(body1)
    grasp_traj = vector_traj_helper(env, robot, approach_vector) # Trajectory from grasp configuration to pregrasp
    #grasp_traj = workspace_traj_helper(base_manip, approach_vector)
    robot.Release(body1)
    if grasp_traj is None: return
    grasp_traj.grasp = grasp
    pregrasp_conf = grasp_traj.end()    # Pregrasp configuration
    yield [(pregrasp_conf, grasp_traj)]

  def sample_free_motion(conf1, conf2): # Sample motion while not holding
    if DISABLE_TRAJECTORIES:
      yield [(object(),)] # [(True,)]
      return
    _enable_all(False)
    set_manipulator_conf(manipulator, conf1.value)
    #traj = motion_plan(env, cspace, conf2.value, self_collisions=True)
    traj = cspace_traj_helper(base_manip, cspace, conf2.value, max_iterations=10)
    if not traj: return
    traj.grasp = None
    yield [(traj,)]

  def sample_holding_motion(conf1, conf2, grasp): # Sample motion while holding
    if DISABLE_TRAJECTORIES:
      yield [(object(),)] # [(True,)]
      return
    _enable_all(False)
    body1.Enable(True)
    set_manipulator_conf(manipulator, conf1.value)
    manip_trans = manipulator.GetTransform()
    set_pose(body1, object_trans_from_manip_trans(manip_trans, grasp.grasp_trans))
    robot.Grab(body1)
    #traj = motion_plan(env, cspace, conf2.value, self_collisions=True)
    traj = cspace_traj_helper(base_manip, cspace, conf2.value, max_iterations=10)
    robot.Release(body1)
    if not traj: return
    traj.grasp = grasp
    yield [(traj,)]

  ####################

  def _execute_traj(traj):
    #for j, conf in enumerate(traj.path()):
    #for j, conf in enumerate([traj.end()]):
    path = list(sample_manipulator_trajectory(manipulator, traj.traj()))
    for j, conf in enumerate(path):
      set_manipulator_conf(manipulator, conf)
      # raw_input('%s/%s) Step'%(j, len(path)))
      print '%s/%s) Step'%(j, len(path))
      wm.getWindow('World').update(); sleep(0.01)

  for s1 in sample_grasp_traj(poses[0], grasps[4]):
    (pre_grasp, traj1) = s1[0]
    set_manipulator_conf(manipulator, initial_conf)
    for s2 in sample_free_motion(env.conf(), pre_grasp):
      (traj2,) = s2[0]
      raw_input('Ok?')
      _execute_traj(traj2)
      _execute_traj(traj1.reverse())    # pick
      raw_input('Continue?')
      print '_cfree_traj_pose(traj, poses[0])', _cfree_traj_pose(traj2, poses[0])
      print '_cfree_traj_grasp_pose(traj, grasps[4], poses[0])', _cfree_traj_grasp_pose(traj2, grasps[4], poses[0])
      print '_cfree_traj_grasp_pose(traj, grasps[4], poses[1])', _cfree_traj_grasp_pose(traj2, grasps[4], poses[1])

  raw_input('Finish?')

##################################################

def main():
  env = Environment()
  simple_test(env)

if __name__ == '__main__':
  main()
  
