from itertools import count
from geometry import hu as hu
from autil.globals import glob
from autil.mmUtil import restFaceIndex
from robot.rrt import runRRT, interpolatePath

def get_window_name(window_3d):
  return window_3d.window.title

def is_window_active(window_3d):
  return window_3d.window is not None

##################################################

def get_name(shape):
  return shape.name()

def get_color(shape):
  return shape.properties['color']

##################################################

def get_current_conf(world_state):
  return world_state.robotConf

def is_left_active(robot):
  return 'pr2LeftArm' in robot.moveChainNames

def is_right_active(robot):
  return 'pr2RightArm' in robot.moveChainNames

def open_grippers(conf):
  for arm in conf.robot.gripperChainNames:
    conf.set(conf.robot.gripperChainNames[arm], [conf.robot.gripMax])

def close_grippers(conf):
  for arm in conf.robot.gripperChainNames:
    conf.set(conf.robot.gripperChainNames[arm], [0])

def set_pose(realWorld, shape, poseValue):
  faceFrame = shape.faceFrames()[restFaceIndex(shape)]
  pose = poseValue.compose(faceFrame.inverse())
  realWorld.setObjectPose(shape.name(), pose)

def grab(realWorld, shape, arm):
  realWorld.attach(get_name(shape), arm)
  realWorld.delObjectState(get_name(shape))

def release(realWorld, arm):
  detached = realWorld.detach(arm)
  realWorld.setObjectPose(get_name(detached), detached.origin().pose())

##################################################

def interpolate(q1, q2, step_size, active_chains=None):
    assert q1.robot == q2.robot
    moveChains = active_chains or q1.keys()
    q = q1
    while not all(q[c] == q2[c] for c in moveChains):
        yield q
        q = q1.robot.stepAlongLine(q2, q, step_size, forward=True, moveChains=moveChains)
    yield q2

def forward_kinematics(conf, arm):
  return conf.cartConf()[conf.robot.armChainNames[arm]]

def inverse_kinematics(default_conf, manip_trans, arm):
  return default_conf.robot.inverseKinWristGen(manip_trans, arm, default_conf,
                                       basePose=default_conf.basePose())

class Traj(object):
  _ids = count(0)
  def __init__(self, path, grasp=None):
    self.id = next(self._ids)
    self.path = path
    self.grasp = grasp
  def __repr__(self):
    return '%s(%d)'%(self.__class__.__name__, self.id)

def arm_motion(realWorld, initial_conf, goal_conf, arm):
  robot = realWorld.robot
  #allowedViol = hu.Violations()
  allowedViol = None # Ends up being the same as hu.Violations()
  path, viol = runRRT(realWorld, initial_conf, goal_conf, allowedViol,
                      [robot.armChainNames[arm]], glob.maxRRTIter, glob.failRRTIter, False)
  if path is None:
    return None
  ipath = interpolatePath([n.conf for n in path])
  #for q in ipath:
  #    place = q.placement()
  #    obstacles = realWorld.objectShapes.values()
  #    collisions = [sh for sh in obstacles if place.collides(sh)]
  #    assert not collisions
  return ipath

##################################################

def get_wrist_frame(grasp, robot, hand):
  gripperFrame = robot.gripperFaceFrame[hand] # Rotates wrist frame to grasp face frame
  wristFrame = grasp.compose(gripperFrame.inverse())
  return wristFrame

def manip_from_pose_grasp(pose, grasp, robot, hand):
  return pose.compose(get_wrist_frame(grasp, robot, hand))

def pose_from_manip_grasp(manip, grasp, robot, hand):
  return manip.compose(get_wrist_frame(grasp, robot, hand).inverse())

def approach_from_manip(manip_trans):
  if abs(manip_trans.matrix[2,0]) < 0.1: # Ad-hoc backoff strategy
    return manip_trans.compose(hu.Pose(-glob.approachBackoff,0.,glob.approachPerpBackoff,0.)) # horizontal
  return manip_trans.compose(hu.Pose(-glob.approachBackoff,0.,0.,0.)) # vertical