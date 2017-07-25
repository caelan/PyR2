import os, copy, yaml, math, time
import geometry.hu as hu
import numpy as np
from collections import deque

from geometry.objects2 import compileChainFramesOS
import geometry.shapes as shapes
import geometry.objects2 as objects
from geometry.objects2 import *
from autil.globals import glob
import geometry.transformations as transf

# Don't reload these files anywhere else!
import robot.conf
import robot.genericRobot
reload(robot.conf)
reload(robot.genericRobot)

from robot.conf import RobotJointConf, RobotCartConf
from robot.genericRobot import GenericRobot

import predefinedBoxLinks

curDir = os.path.dirname(os.path.abspath(__file__))
urdfPath = curDir + '/draper_ptu_gripper.urdf'
conf_file = curDir + '/iiwa.yaml'
config = yaml.load(file(conf_file,'r'))

# This specified a joint configuartion, specifying the joint angles for all the chains.
robotInit, robotChainNames, chainConstraint = {}, [], []
viewDistanceConstraint = config['robotChains']['robotHead']['viewingDistanceConstraint']

################################################################
# !! Jay: The following gripper frames and offsets have to be more
# made in a more generic way
################################################################
Ident = hu.Transform(np.eye(4, dtype=np.float64)) # identity transform

def vec(str): return [float(x) for x in str.split()]
def transProd(lt): return hu.Transform(reduce(np.dot, lt))
def Ba(bb, **prop): return shapes.BoxAligned(np.array(bb), Ident, **prop)
def Sh(*args, **prop): return shapes.Shape(list(args), Ident, **prop)

def setEpsilonToZero(x, epsilon):
    for i in xrange(0, len(x)):
        if abs(i) < epsilon: x[i] = 0.0
    return x

def setMagnitudeBounds(x, magnitude):
    return min(max(x, -abs(magnitude)), abs(magnitude))

gripper_offset = 0.175
# Rotates wrist to grasp face frame
gFaceFrame = hu.Transform(np.array([(0.,1.,0.,gripper_offset),
                                    (0.,0.,1.,0.),
                                    (1.,0.,0.,0.),
                                    (0.,0.,0.,1.)]))

#gFaceFrame.matrix = np.dot(gFaceFrame.matrix, transf.euler_matrix(0, 0, 0))

gripperFaceFrame = {'left': gFaceFrame, 'right': gFaceFrame}

palm_dx, palm_dy, palm_dz = config['gripperParams']['palm']
fing_dx, fing_dy, fing_dz = config['gripperParams']['finger']

def robotGripperLinks():
    return [Sh(shapes.Box(palm_dx, palm_dy, palm_dz, Ident, name='palm')),
            Sh(shapes.Box(fing_dx, fing_dy, fing_dz, Ident, name='finger1')),
            Sh(shapes.Box(fing_dx, fing_dy, fing_dz, Ident, name='finger2'))]

def robotGripperJoints(arm):
    return [Rigid(arm+'_palm', hu.Transform(transf.euler_matrix(-math.pi, 0, 0)), None, None),
            Prismatic(arm+'_finger1',
                      hu.Transform(transf.translation_matrix([palm_dx/2+fing_dx/2.,fing_dy/2.,0.0])),
                      (0.0, 0.05/2.), (0.,1.,0)),
            Prismatic(arm+'_finger2',
                      hu.Transform(transf.translation_matrix([0.0,-fing_dy,0.0])),
                      (0.0, 0.081), (0.,-1.,0))]

################################################################
# Conf and CartConf, mostly generic
################################################################

# This behaves like a dictionary, except that it doesn't support side effects.
class JointConf(RobotJointConf):
    def __init__(self, conf, robot):
        RobotJointConf.__init__(self, conf, robot)
    def copy(self):
        return JointConf(self.conf.copy(), self.robot)

# In a cartesian configuration, we specify frames for base, hand and head
class CartConf(RobotCartConf):
    def __init__(self, conf, robot):
        RobotCartConf.__init__(self, conf, robot)
    def copy(self):
        return CartConf(self.conf.copy(), self.robot)

robotColor = config['robotColor']
robotBaseLink = predefinedBoxLinks.predefined_links(config['robotChains']['robotBase']['predefinedLinks'])

robotChains = {}
def makeRobotChains(name, workspaceBounds, new=True):
    global robotChains
    if not new and name in robotChains:
         return robotChains[name]

    chains = []

    for c in config['chainBuildOrder']:
        for chainType in config['robotChains']:
            if chainType == c:
                #print '[BUILDING ROBOT] Adding parameters into initiallizer for,', chainType
                robotInit[str(chainType)] = config['robotChains'][chainType]['defaultConf']
                robotInit[str(chainType)] = [float(i) for i in robotInit[str(chainType)]]   #ensure that these are all floats

                robotChainNames.append(chainType)

                #print '[BUILDING ROBOT] Creating chain,', chainType
                if chainType == 'robotBase':
                    try: # robotBase chain creation -- defined as the base_link plane.
                        baseName = config['robotChains']['robotBase']['chainName']
                        baseChain = Planar(baseName, 'root', robotBaseLink, workspaceBounds)
                        chains.append(baseChain)
                    except: assert None, 'ERROR: A base chain must be specified by the config file!'

                elif 'Gripper' in chainType:
                    rightGripperChain = GripperChain('robotRightGripper', config['robotChains'][chainType]['jointMount'],
                                                     robotGripperJoints('r'), robotGripperLinks())
                    chains.append(rightGripperChain)
                else:
                    joints = config['robotChains'][chainType]['frames']
                    links = predefinedBoxLinks.predefined_links(config['robotChains'][chainType]['predefinedLinks'])
                    newChain = Chain(chainType, config['robotChains'][chainType]['jointMount'],
                                     getUrdfJoints(joints, urdfPath),
                                     links)
                    chains.append(newChain)

    # Create robot which is a multichain of subchains
    robotChains[name] = MultiChain(name, chains)
    return robotChains[name]


robotChainDependencies, robotChainDependenciesRev = {}, {}
for chainName in config['robotChains']:
    robotChainDependencies[chainName] = [chainName] # Assume self dependencies...?
    robotChainDependenciesRev[chainName] = []
    try:
        for extra_deps in config['robotChains'][chainName]['depends']:
            robotChainDependencies[chainName].append(extra_deps)
    except: pass
    try:
        for rev_deps in config['robotChains'][chainName]['dependsRev']:
            robotChainDependenciesRev[chainName].append(rev_deps)
    except: pass

# Transforms from wrist frame to hand frame
# This leaves X axis unchanged
right_gripperToolOffsetX = hu.Pose(gripper_offset,0.0,0.0,0.0)
# This rotates around the Y axis... so Z' points along X an X' points along -Z
right_gripperToolOffsetZ = hu.Transform(np.dot(right_gripperToolOffsetX.matrix,
                                                 transf.rotation_matrix(math.pi/2,(0,1,0))))

class IIWARobot(GenericRobot):
    def __init__(self, name, chains, color = robotColor):
        self.chains = chains
        self.color = color
        self.name = name
        self.useLeft = False
        self.useRight = True

        self.scanner = config['scannerParams'] # Kinect(focal, height, width, length, n)

        self.chainNames = robotChainNames
        self.bodyChains, self.moveChainNames = [], []
        self.baseChainName, self.headChainName, self.torsoChainName = [], [], []
        self.armChainNames, self.gripperChainNames, self.wristFrameNames = {}, {}, {}
        self.selfCollideChainNames = [[], [], [], [], []]

        for chain in config['robotChains']:
            if 'robotBase' in config['robotChains']:
                self.baseChainName = 'robotBase'
                self.bodyChains.append('robotBase')
            if 'robotHead' in config['robotChains']:
                self.headChainName = 'robotHead'
                self.bodyChains.append('robotHead')
            if 'robotTorso' in config['robotChains']:
                self.torsoChainName = 'robotTorso'
                self.bodyChains.append('robotTorso')
            if 'Arm' in chain:
                self.armChainNames[config['robotChains'][chain]['armOrient']] = config['robotChains'][chain]['chainName']
                self.moveChainNames.append(config['robotChains'][chain]['chainName'])
            if 'Gripper' in chain:
                self.wristFrameNames[config['robotChains'][chain]['armOrient']] = config['robotChains'][chain]['wristName']
                self.gripperChainNames[config['robotChains'][chain]['armOrient']] = config['robotChains'][chain]['chainName']

        # TLP - What should this be?
        # This has the X axis pointing along fingers
        self.toolOffsetX = {'right': right_gripperToolOffsetX}
        # This has the Z axis pointing along fingers (more traditional, as in ikFast)
        self.toolOffsetZ = {'right': right_gripperToolOffsetZ}

        self.gripperFaceFrame = gripperFaceFrame

        # TLP - what should this be?
        self.gripMax = 0.08 

        self.previousHeadConf = config['robotChains']['robotHead']['defaultConf']
        self.nominalConf = None
        self.confCache = {}
        self.confCacheKeys = deque([])  # in order of arrival

        #print self.chains.chainsInOrder
        self.compiledChainsOS = compileChainFramesOS(self) # generic
        self.chainDependencies = robotChainDependencies
        self.chainDependRev = robotChainDependenciesRev
        self.OSa = None

    # hands are associated with Grippers
    def handNames(self):
        return [config['robotChains'][chain]['armOrient'] for chain in config['robotChains'] if 'Gripper' in chain]

    def clearInvKinCache(self):
        time.sleep(0.01)
        pass

    def makeJointConf(self, confDict):
        return JointConf(confDict, self)

    def makeCartConf(self, conf):
        return CartConf(conf, self)

    # This is useful to get the base shape when we don't yet have a conf
    def baseLinkShape(self, basePose=None):
        if basePose: return robotBaseLink.applyTrans(basePose)
        else: return robotBaseLink

    # This was taken directly from pr2Robot.. TODO: generalize
    def fingerSupportFrame(self, hand, width):
        # The old way...
        # Origin is on the inside surface of the finger (at the far tip).
        # The -0.18 is from finger tip to the wrist  -- if using wrist frame
        # mat = np.dot(transf.euler_matrix(-math.pi/2, math.pi/2, 0.0, 'ryxz'),
        #              transf.translation_matrix([0.0, -0.18, -width/2]))

        # y points along finger approach, z points in closing direction
        # offset aligns with the grasp face.
        # This is global gripperFaceFrame offset to center object

        # TLP -- why this?
        # matrix = np.dot(transf.euler_matrix(math.pi/2., math.pi/2., 0),transf.translation_matrix([0, fing_dx + 0.025, 0.]))
        # return hu.Transform(matrix)
    
        gripperFaceFrame_dy = hu.Transform(np.array([(0.,1.,0., gripper_offset),
                                                     (0.,0.,1.,-width/2),
                                                     (1.,0.,0.,0.),
                                                     (0.,0.,0.,1.)]))
        return gripperFaceFrame_dy

    # Note that the "ArmFrame" is the wrist frame.
    def forwardKin(self, conf, complain = False, fail = False):
        shapes, frames = self.placement(conf, getShapes=[])
        cartDict = {}
        for chainName in config['robotChains']:
            # TLP - GripperFrame does not exist...
            if 'Gripper' in chainName:
                cartDict[chainName] = conf[chainName]
            else:
                cartDict[str(chainName)+'Frame'] = frames[self.chains.chainsByName[chainName].joints[-1].name]
        return CartConf(cartDict, self)

    # This is a "partial" inverse kinematics involving only one hand
    # and (optionally) a basePose and/or grip.  The defaultConf
    # specifies the other parameters.  In the missing basePose case,
    # it should generate alternatives.
    #
    # Jay: It looks like the inputs are as follows,
    #   wrist: hu.Transform
    #   hand: {left, right, etc}
    #   defaultConf: conf (configuration), pass this as conf to IK
    #   basePose: a constraint in Cartesian (can be None)
    #   grip: a grip constraint
    def inverseKinWristGen(self, wrist, hand, defaultConf, basePose=None, grip=None,
                                 complain=True, n=None, counts=None, provideError=False):

        # TLP - What is this?  Modifying the wrist matrix creates terrible side-effects!
        # if grip:
        #     wrist.matrix = np.dot(wrist.matrix, transf.euler_matrix(math.pi, 0.0, 0.0))
        #     wrist.matrix = np.dot(wrist.matrix, transf.translation_matrix([grip, 0.0, 0.0]))

        
        cc = {'robotRightArmFrame': copy.copy(wrist)} # TLP - was GripperFrame
        if basePose: cc['robotBaseFrame'] = basePose

        cart = self.makeCartConf(cc)
        conf = self.inverseKin(cart, defaultConf)
        #raw_input('Continue?')

        if conf and all(conf.conf.values()):
            yield conf

    def inverseKin(self, cart, conf, returnAll = False, complain = False, fail = False, provideError=False, doPrimitive=False):
        initConf = self.trivialInverseKin(cart, conf)
        conf = self.inverseKinGeneric(cart, initConf, maxRestarts=glob.maxRestartsGenIK)
        if all(conf.conf.values()):
            # conf.draw('W', 'cyan')
            # raw_input('Go?')
            return conf

    def trivialInverseKin(self, cart, conf):
        chainName = 'robotHead'
        if not cart.get(chainName):
            return conf
        pitch, yaw = None, None
        try: # If it succeeds then likely target was specified as a hu.Pose
            targetPose = cart.get(chainName, None).xyztTuple()
        except: # In this case this is specified through a hu.Transform
            targetPose = cart.get(chainName, None)
        assert targetPose
        currentCartPose = self.forwardKin(conf)
        currentFramePose = currentCartPose.get(chainName, None)
        currentFramePoint = transf.translation_from_matrix(currentFramePose.matrix)
        dX = targetPose[0]-currentFramePoint[0]
        dY = targetPose[1]-currentFramePoint[1]
        dZ = targetPose[2]-currentFramePoint[2]
        view_dist = np.linalg.norm(list(targetPose[0:2])-currentFramePoint[0:2])
        pitch = -math.atan2(-dZ, view_dist) - 0.0 # We might want to offset the pitch a tiny bit
        yaw = -math.atan2(dY, dX)
        if yaw: # if there is some solution for the head
            pitch = setMagnitudeBounds(pitch, 0.5) # !! Jay, don't get it go too large
            yaw = setMagnitudeBounds(yaw, 2.0) # !! Jay, don't get it go too large
            conf = conf.set('robotHead', [yaw, pitch])
            self.previousHeadConf = [yaw, pitch]
        else:
            conf = conf.set('robotHead', self.previousHeadConf)

        if config['fixedBase']:
            baseAngles = config['robotChains']['robotBase']['defaultConf']
            conf = conf.set('robotBase', baseAngles)

        return conf

    def makeConf(robot, x, y, th=0.0, g=0.07, vertical=None):
        c = JointConf(robotInit.copy(), robot)
        c = c.set('robotBase', [x, y, th])
        return c

def makeRobot(workspace, radiusVar=0.0, useLeft=False, useRight=True):
    robot_name = config['robotName']
    chains = makeRobotChains(robot_name, workspace)
    robot = IIWARobot(robot_name, chains) # uses right
    robot.nominalConf = JointConf(robotInit, robot)
    robot.moveChainNames = []
    if useRight:
        robot.moveChainNames.extend(['robotRightArm', 'robotRightGripper'])
    return robot
