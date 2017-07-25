import pdb
import math, string, inspect
from itertools import product
from random import sample
import numpy as np

from autil.globals import glob
from autil.mmUtil import objectGraspFrameAux, restFaceIndex, GDesc
from autil.miscUtil import Hashable
import graphics.windowManager3D as wm

import geometry.hu as hu
import geometry.shapes as shapes
from geometry.objects2 import World, Universe, WorldState
from geometry.transformations import euler_from_quaternion

from robot.rrt import runRRT, interpolatePath

# These are PR2 specific, could be replaced with a different robot
from robot.pr2.pr2Robot import makeRobot
glob.usePR2(big_workspace=True)

##################################################
# Interface to BHPN
##################################################

universe = Universe()
class Environment:
    def __init__(self):
        self.world = World(universe)
        self.arm = 'left'
        robot = makeRobot(np.array(glob.workspace), useRight=False)
        self.world.setRobot(robot)
        self.robot = self.world.robot
        self.realWorld = WorldState(self.world, self.robot)
        self.grasps = {}
        self.lock = True
    def Add(self, body):                # Body instance
        self.world.addObjectShape(body.shape)
        self.set_pose(body, hu.Pose(0.,0.,0., 0.))
    def GetRobots(self):
        return [Robot(self)]
    def GetViewer(self):
        wm.makeWindow('World', glob.viewPort, glob.windowSizes.get('World', 500))
        wm.makeWindow('W', glob.viewPort, glob.windowSizes.get('W', 800))
        return True
    def GetKinBody(self, objName):            # objName is string
        return Body(self, self.bodies()[objName])
    def Load(self, dir):
        print 'env.Load called with', dir
    def Lock(self):
        self.lock = True
        return
    def Unlock(self):
        self.lock = False               # allow displays
        wm.getWindow('W').clear(); wm.getWindow('World').clear()
        return
    def UpdatePublishedBodies(self):
        return

    def set_arm(self, arm):
        self.arm = 'left' if arm[0] == 'l' else 'right'
    def set_default_robot_config(self):
        baseConf = (self.conf().value or self.robot.nominalConf).baseConf()
        conf = self.realWorld.robot.makeConf(*baseConf, ignoreStandard=True)
        self.realWorld.setRobotConf(conf) # instances of Manipulator and Conf
        if not self.lock:
            wm.getWindow('World').clear()
            self.realWorld.draw('World')
    def set_manipulator_conf(self, manipulator, confVal):
        if isinstance(confVal, list):
            chain = self.robot.armChainNames[self.arm]
            confVal = self.conf().value.set(chain, confVal)
        # print 'set_manipulator_conf', confVal
        self.realWorld.setRobotConf(confVal)
        if not self.lock:
            wm.getWindow('World').clear()
            self.realWorld.draw('World')
    def set_base_values(self, robot, baseConf):
        self.realWorld.setRobotConf(self.realWorld.robotConf.setBaseConf(baseConf))
    def set_pose(self, body, poseValue):
        # print 'set_pose', body.shape.name(), poseValue.pose(fail=False)
        faceFrame = body.shape.faceFrames()[restFaceIndex(body.shape)]
        pose = poseValue.compose(faceFrame.inverse())
        self.realWorld.setObjectPose(body.shape.name(), pose)
        if not self.lock:
            wm.getWindow('World').clear()
            self.realWorld.draw('World')
    def bodies(self):
        return self.realWorld.objectShapes
    def conf(self):
        return Conf(self.realWorld.robotConf)
    def CheckCollision(self, body1, body2):
        body2Name = body2.shape.name()
        if isinstance(body1, Robot):
            robotShape = self.realWorld.robotConf.placement()
            return robotShape.collides(self.bodies()[body2Name])
        else:
            body1Name = body1.shape.name()
            return self.bodies()[body1Name].collides(self.bodies()[body2Name])

##################################################
# Types used in StripStream TAMP
##################################################

class Pose(Hashable):
    def __init__(self, pose):
        self.value = pose
        Hashable.__init__(self)
    def desc(self):
        return self.value.pose().xyztTuple()

class Grasp(Hashable):
    def __init__(self, objName, grasp, trans, approach=None):
        self.value = grasp
        self.objName = objName
        self.grasp_trans = trans
        self.approach_vector = approach
        Hashable.__init__(self)
    def desc(self):
        return (self.objName, self.value)

class Conf(Hashable):
    def __init__(self, conf):
        self.value = conf
        Hashable.__init__(self)
    def __getitem__(self, name):
        return self.value[name]
    def desc(self):
        return tuple(self.value) if isinstance(self.value, list) else self.value

class Traj(Hashable):
    def __init__(self, path, grasp=None):
        self.pathConfs = tuple(path)
        self.grasp = grasp
        Hashable.__init__(self)
    def path(self):
        return self.pathConfs
    def traj(self):
        return self.pathConfs
    def end(self):
        return self.pathConfs[-1]
    def reverse(self):
        return Traj(self.pathConfs[::-1], self.grasp)
    def desc(self):
        return (self.pathConfs, self.grasp)

class Body:
    def __init__(self, env, shape):
        self.shape = shape              # shape at origin
        self.enabled = False
        self.point = None
        self.env = env
    def Enable(self, value):
        self.enabled = value
    def GetKinematicsGeometryHash(self):
        return None                     # TODO

class Robot:
    def __init__(self, env):
        self.env = env
    def GetActiveManipulator(self):
        return Manipulator(self, self.env.arm)
    def GetConfigurationValues(self):
        return self.env.conf()
    def Grab(self, body):
        env = self.env
        realWorld = env.realWorld
        objName = body.shape.name()
        realWorld.attach(objName, env.arm)
        realWorld.delObjectState(objName)
    def Release(self, body):
        env = self.env
        realWorld = env.realWorld
        objName = body.shape.name()
        detached = realWorld.detach(env.arm)
        realWorld.setObjectPose(detached.name(), detached.origin().pose())

class Manipulator:
    def __init__(self, robot, arm):
        self.robot = robot
        self.robot.env.set_arm(arm)
    def GetArmIndices(self):
        return self.robot.env.robot.armChainNames[self.robot.env.arm]
    def GetTransform(self):
        return self.robot.env.conf().value.cartConf()[self.GetArmIndices()]

class CSpace:                   # dummy
    @staticmethod
    def robot_arm(manipulator):
        return manipulator

class interfaces:               # dummy
    @staticmethod
    def BaseManipulation(robot, plannername=None, maxvelmult=None):
        return robot

##################################################
# Functions used in StripStream TAMP
##################################################

def initialize_openrave(env, arm, min_delta=.01):
    env.set_arm(arm)
    env.Unlock()                # make sure we draw
    if env.arm == 'left': 
        glob.useRight = False; glob.useLeft = True
    else:
        glob.useRight = True; glob.useLeft = False
    env.set_default_robot_config()
    env.min_delta = min_delta

def get_grasps(env, robot, body1, use_grasp_approach, use_grasp_type):
    return env.grasps[body1.shape.name()]

def open_gripper(manipulator):
    env = manipulator.robot.env
    nconf = env.conf().value.set(env.robot.gripperChainNames[env.arm],
                                 [env.robot.gripMax])
    set_manipulator_conf(manipulator, Conf(nconf))

def set_default_robot_config(robot): # instance of Robot
    robot.env.set_default_robot_config()

def set_point(body, point):
    body.point = point

def get_point(body):
    return body.point

def set_pose(body, pose):
    poseValue = pose.value if isinstance(pose, Pose) else pose
    # Might need to center in z (and x,y?)
    body.env.set_pose(body, poseValue)

def set_base_values(robot, base):
    robot.env.set_base_values(robot, base)
set_base_conf = set_base_values

def set_manipulator_conf(manipulator, conf):
    confValue = conf.value if isinstance(conf, Conf) else conf
    manipulator.robot.env.set_manipulator_conf(manipulator, confValue)

def object_trans_from_manip_trans(t1, t2):
    return Pose(t1.compose(t2.inverse()))

def get_name(body):
    return body.shape.name()

def get_color(color):
    return color

def _enable_all(val):
    return

def manip_from_pose_grasp(pose, grasp):
    # This is the robot wrist
    manip_trans = pose.value.compose(grasp.grasp_trans)
    # Ad-hoc backoff strategy
    if abs(manip_trans.matrix[2,0]) < 0.1: # horizontal
        offset = hu.Pose(-glob.approachBackoff,0.,glob.approachPerpBackoff,0.)
    else:                               # vertical
        offset = hu.Pose(-glob.approachBackoff,0.,0.,0.)
    manip_trans_approach = manip_trans.compose(offset)
    # The grasp and the approach
    return manip_trans, manip_trans_approach

def solve_inverse_kinematics(env, manipulator, manip_trans):
    robot = env.robot
    hand = env.arm
    conf = env.conf().value
    for conf in robot.inverseKinWristGen(manip_trans, hand, conf,
                                         basePose=conf.basePose()):
        return conf
    print None

def vector_traj_helper(env, robot, approach_trans): # Trajectory from grasp configuration to pregrasp
    approach_conf = solve_inverse_kinematics(env, robot, approach_trans)
    # robot is currently at grasp conf, trajectory goes to pre-conf,
    # should be linear intepolation, but just joint interpolation now.
    path = interpolatePath([env.conf().value, approach_conf])
    return Traj([Conf(q) for q in path])

def cspace_traj_helper(base_manip, cspace, conf2Value, max_iterations=100):
    # cspace is a manipulator...
    env = cspace.robot.env
    initConf = env.conf().value
    destConf = conf2Value
    allowedViol = hu.Violations()
    moveChains = [cspace.GetArmIndices()]
    maxIter = glob.maxRRTIter
    failIter = glob.failRRTIter
    optimize = True
    path, viol = runRRT(env.realWorld, initConf, destConf, allowedViol,
                        moveChains, maxIter, failIter, False)
    ipath = interpolatePath([n.conf for n in path])
    for q in ipath:
        place = q.placement()
        obstacles = env.realWorld.objectShapes.values()
        collisions = [sh for sh in obstacles if place.collides(sh)]
        if collisions:
            wm.getWindow('W').clear(); env.realWorld.draw('W')
            q.draw('W', 'red');
            for sh in collisions: sh.draw('W', 'red')
            print 'Collision!'
            pdb.set_trace()
    return Traj([Conf(q) for q in ipath]) if ipath else None

def sample_manipulator_trajectory(manipulator, traj):
    return traj.path() if isinstance(traj, Traj) else traj

##################################################
# Making box bodies
##################################################

def Ba(bb, **prop): return shapes.BoxAligned(np.array(bb), None, **prop)
def Sh(args, **prop): return shapes.Shape(list(args), None, **prop)

# Grasps
# from the side
# the z offset raises or lowers the grasp relative to midpoint of object
gMat0 = hu.Transform(np.array([(0.,1.,0.,0.),
                               (0.,0.,1.,-0.025),
                               (1.,0.,0.,0.02),
                               (0.,0.,0.,1.)]))
gMat0h = hu.Transform(np.array([(0.,1.,0.,0.0),
                               (0.,0.,1.,-0.025),
                               (1.,0.,0.,0.05),
                               (0.,0.,0.,1.)]))
gMat1 = hu.Transform(np.array([(0.,-1.,0.,0.),
                               (0.,0.,-1.,0.025),
                               (1.,0.,0.,0.02),
                               (0.,0.,0.,1.)]))
gMat1h = hu.Transform(np.array([(0.,-1.,0.,0.),
                               (0.,0.,-1.,0.025),
                               (1.,0.,0.,0.05),
                               (0.,0.,0.,1.)]))
gMat4 = hu.Pose(0,0,0,math.pi/2).compose(gMat0)
gMat5 = hu.Pose(0,0,0,-math.pi/2).compose(gMat0)
# from the top
gMat2= hu.Transform(np.array([(-1.,0.,0.,0.),
                              (0.,0.,-1.,0.025),
                              (0.,-1.,0.,0.),
                              (0.,0.,0.,1.)]))
gMat3= hu.Transform(np.array([(1.,0.,0.,0.),
                              (0.,0.,1.,-0.025),
                              (0.,-1.,0.,0.),
                              (0.,0.,0.,1.)]))

gdesc0 = lambda obj: GDesc(obj, gMat0, 0.05, 0.05, 0.025)
gdesc0h = lambda obj: GDesc(obj, gMat0h, 0.05, 0.05, 0.025)
gdesc1 = lambda obj: GDesc(obj, gMat1, 0.05, 0.05, 0.025)
gdesc1h = lambda obj: GDesc(obj, gMat1h, 0.05, 0.05, 0.025)
gdesc2 = lambda obj: GDesc(obj, gMat2, 0.05, 0.05, 0.025)
gdesc3 = lambda obj: GDesc(obj, gMat3, 0.05, 0.05, 0.025)
gdesc4 = lambda obj: GDesc(obj, gMat4, 0.05, 0.05, 0.025)
gdesc5 = lambda obj: GDesc(obj, gMat5, 0.05, 0.05, 0.025)

BLUE = 'blue'
RED = 'red'
colors = ['red', 'green', 'blue', 'cyan', 'purple', 'pink', 'orange']

def pickColor(name):
    if name[-1] in string.uppercase:
        cn = len(colors)
        return colors[string.uppercase.index(name[-1])%cn]
    else:
        return 'black'

def makeBox(dx=0.025, dy=0.025, dz=0.1, name='boxA', color=None):
    gds = []
    if glob.useHorizontal:
        gds.extend([gdesc0(name), gdesc1(name), gdesc4(name), gdesc5(name)])
    if glob.useVertical:
        gds.extend([gdesc2(name), gdesc3(name)])
    universe.setObjectTypeData('box', 'graspDesc', gds)
    color = color or pickColor(name)
    return (Sh([Ba([(-dx, -dy, 0.), (dx, dy, dz)])], name=name, color=color), [])
universe.setObjectType('box')
universe.setObjectType('goal', 'box')
universe.setObjectType('block', 'box')
universe.setObjectType('table', 'box')
universe.setObjectTypeData('box', 'constructor', makeBox)

def box_body(env, length, width, height, name='box', color='black'):
    def trans(graspIndex):
        wrist = objectGraspFrameAux(gd, graspIndex, graspMu,
                                    faceFrames, restFace, Ident,
                                    robot, hand)
        return wrist
    shape, _ = env.world.getObjectTypeData('box', 'constructor')(dx=length/2., dy=width/2., dz=height, name=name)
    if length <= 0.07 and width <= 0.07:  # graspable
        gd = env.world.getObjectTypeData(name, 'graspDesc', [])
        hand = env.arm
        robot = env.robot
        graspMu = hu.Pose(0.0, -0.025, 0.0, 0.0) # Note !! constant
        faceFrames = shape.faceFrames()
        restFace = restFaceIndex(shape)
        Ident = hu.Pose(0,0,0,0)
        env.grasps[name] = [Grasp(name, i, trans(i)) for i,_ in enumerate(gd)]
    else:
        env.grasps[name] = []
    return Body(env, shape)

##################################################
# Creating problems
##################################################

class ManipulationProblem:
    def __init__(self, name,
                 object_names=[], table_names=[], initial_poses={},
                 goal_poses={}, known_poses=[]):
        self.name = name
        self.object_names = object_names
        self.table_names = table_names
        self.initial_poses = initial_poses
        self.goal_poses = goal_poses
        self.known_poses = known_poses

BODY_PLACEMENT_Z_OFFSET = 1e-3
REARRANGEMENT = True
ENVIRONMENTS_DIR = './'

def function_name(stack): # NOTE - stack = inspect.stack()
  return stack[0][3]
def flatten(x):
    xflat = []
    for xi in x: xflat.extend(xi)
    return xflat
def unit_quat(): return np.array([0,0,0,1])
def pose_from_quat_point(quat, xyz):
    angle = euler_from_quaternion(quat)[2] # z rotation
    return hu.Pose(xyz[0], xyz[1], xyz[2], angle)

##################################################
# Problem definitions
##################################################

# This is taken verbatim from manipulation.problems.distribution
def dantam2(env): # (Incremental Task and Motion Planning: A Constraint-Based Approach)
  assert REARRANGEMENT
  env.Load(ENVIRONMENTS_DIR + 'empty.xml')

  m, n = 3, 3
  #m, n = 5, 5
  n_obj = 8
  side_dim = .07 # .05 | .07
  height_dim = .1
  box_dims = (side_dim, side_dim, height_dim)
  separation = (side_dim, side_dim)
  #separation = (side_dim/2, side_dim/2)

  coordinates = list(product(range(m), range(n)))
  assert n_obj <= len(coordinates)
  obj_coordinates = sample(coordinates, n_obj)

  length = m*(box_dims[0] + separation[0])
  width = n*(box_dims[1] + separation[1])
  height = .7
  table = box_body(env, length, width, height, name='table', color=get_color('brown'))
  set_point(table, (0, 0, 0))
  env.Add(table)

  robot = env.GetRobots()[0]
  set_default_robot_config(robot)
  set_base_values(robot, (-1.5, 0, 0))
  #set_base_values(robot, (0, width/2 + .5, math.pi))
  #set_base_values(robot, (.35, width/2 + .35, math.pi))
  #set_base_values(robot, (.35, width/2 + .35, 3*math.pi/4))

  poses = []
  z =  get_point(table)[2] + height + BODY_PLACEMENT_Z_OFFSET
  for r in range(m):
    row = []
    x = get_point(table)[0] - length/2 + (r+.5)*(box_dims[0] + separation[0])
    for c in range(n):
      y = get_point(table)[1] - width/2 + (c+.5)*(box_dims[1] + separation[1])
      row.append(Pose(pose_from_quat_point(unit_quat(), np.array([x, y, z]))))
    poses.append(row)

  initial_poses = {}
  goal_poses = {}
  # TODO - randomly assign goal poses here
  for i, (r, c) in enumerate(obj_coordinates):
    row_color = np.zeros(4)
    row_color[2-r] = 1.
    if i == 0:
      name = 'goal%d-%d'%(r, c)
      color = BLUE
      goal_poses[name] = poses[m/2][n/2]
    else:
      name = 'block%d-%d'%(r, c)
      color = RED
    initial_poses[name] = poses[r][c]
    obj = box_body(env, *box_dims, name=name, color=color)
    env.Add(obj)                # needs Add and set_pose...
    set_pose(obj, poses[r][c].value)


  #for obj in randomize(objects):
  #  randomly_place_body(env, obj, [get_name(table)])

  known_poses = list(flatten(poses))
  #known_poses = list(set(flatten(poses)) - {poses[r][c] for r, c in obj_coordinates}) # TODO - put the initial poses here

  return ManipulationProblem(
      function_name(inspect.stack()),
      object_names=initial_poses.keys(), table_names=[get_name(table)],
      goal_poses=goal_poses,
      initial_poses=initial_poses, known_poses=known_poses)


  
