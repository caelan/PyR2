import math
from collections import deque

import geometry.hu as hu
import numpy as np
import geometry.shapes as shapes

import geometry.transformations as transf
from geometry.chains import compileChainFramesOS, Chain, MultiChain, getUrdfJoints, GripperChain, Planar, Rigid, Prismatic
from geometry.pr2.pr2IkPoses import ikTrans # base poses
from geometry.pr2.pr2IkMap import ikTransLRz  # (z, base pose)
from geometry.pr2.ik2 import armInvKin2, clearInvKinCache2

# Don't reload these files anywhere else!
import geometry.conf
import geometry.genericRobot
reload(geometry.conf)
reload(geometry.genericRobot)

from geometry.conf import RobotJointConf, RobotCartConf
from geometry.genericRobot import GenericRobot

Ident = hu.Transform(np.eye(4, dtype=np.float64)) # identity transform

import os
curDir = os.path.dirname(os.path.abspath(__file__))
urdfPath = curDir + '/pr2.urdf'

standardVerticalConf = None
standardHorizontalConf = None

TORSO_Z = 0.3
BASE_GROWTH_X = 0.1
BASE_GROWTH_Y = 0.1
USE_OLD_IK_POSES = True

################################################################
# Define the robot, ideally would be from a URDF.
################################################################

def vec(str):
    return [float(x) for x in str.split()]

def transProd(lt):
    return hu.Transform(reduce(np.dot, lt))

# Force sensor offsets from Gustavo's URDF

# <origin rpy="0 -1.5707963267948966 0" xyz="0.0356 0 0"/>
# <origin rpy="0 0 1.0477302122478542" xyz="0 0 0"/>
# <origin rpy="0 1.5707963267948966 -1.2217304763960306" xyz="0 0 0"/>

r_forceSensorOffset = transProd([transf.translation_matrix(vec("0.0356 0 0")),
                                 transf.euler_matrix(*vec("0 -1.5707963267948966 0"), axes='sxyz'),
                                 transf.euler_matrix(*vec("0 0 1.0477302122478542"), axes='sxyz'),
                                 transf.euler_matrix(*vec("0 1.5707963267948966 -1.2217304763960306"), axes='sxyz')])
l_forceSensorOffset = Ident

# 'beige' for movies, but that doesn't show up against white background of 2D
# Python simulator.
pr2Color = 'gold'

pr2_torso_joints = ['base_footprint_joint',
                    'torso_lift_joint']

pr2_arm_joints = ['_shoulder_pan_joint',
                  '_shoulder_lift_joint',
                  '_upper_arm_roll_joint',
                  '_elbow_flex_joint',
                  '_forearm_roll_joint',
                  '_wrist_flex_joint',
                  '_wrist_roll_joint']

def armJointNames(arm, joint_names=pr2_arm_joints):
    return [arm+j for j in joint_names]

pr2_head_joints = ['head_pan_joint',
                   'head_tilt_joint',
                   'head_plate_frame_joint',
                   'head_mount_joint',
                   'head_mount_kinect_ir_joint',
                   'head_mount_kinect_ir_optical_frame_joint'
                   ]

def Ba(bb, **prop): return shapes.BoxAligned(np.array(bb), Ident, **prop)
def Sh(*args, **prop): return shapes.Shape(list(args), Ident, **prop)

# Small base and torso
pr2BaseLink = Sh(\
    Ba([(-0.43, -0.33, 0.0), (0.33, 0.33, 0.33)], name='base'),
    Ba([(-0.43, -0.33, 0.33), (0.1, 0.33, 1.0)], name='torso')
    )

# More conservative (bigger) base and torso
pr2BaseLinkGrown = Sh(\
    Ba([(-0.43-BASE_GROWTH_X, -0.33-BASE_GROWTH_Y, 0.0), (0.33+BASE_GROWTH_X, 0.33+BASE_GROWTH_Y, 0.33)], name='baseGrown'),
    Ba([(-0.43-BASE_GROWTH_X, -0.33-BASE_GROWTH_Y, 0.0), (0.1, 0.33+BASE_GROWTH_Y, 1.5)], name='torsoGrown')
    )

# Connects to base, depends on torso height
pr2TorsoLinks = [\
    None,
    Sh(Ba([(-0.1, -0.1, 0.1), (0.1, 0.1, 0.3)], name='neck'))
    ]

# Connectz to torso, depens on head angles
pr2HeadLinks = [\
    None,
    # Sh(Ba([(0, -0.01, -0.01), (0.1, 0.01, 0.01)], name='sensorX'),
    #    Ba([(-0.01, 0, -0.01), (0.01, 0.1, 0.01)], name='sensorY'),
    #    Ba([(-0.01, -0.01, 0), (0.01, 0.01, 0.1)], name='sensorZ')),
    None,
    Sh(Ba([(-0.2, -0.1, -0.05), (0.1, 0.1, 0.05)], name='head')),
    None,
    None,
    Sh(Ba([(-0.1, -0.025, -0.05), (0.1, 0.025, 0.05)], name='kinect'))
       # a "beam" from the center of the kinect, along Z axis
       # Ba([(-0.01, -0.01, 0), (0.01, 0.01, 2)], name='sensorBeam')
    ]

def pr2ArmLinks(arm):
    angle = math.pi/2 if arm=='r' else -math.pi/2
    pose = hu.Transform(transf.rotation_matrix(angle, (1,0,0)))
    links = [\
        Sh(Ba([(-0.10, -0.12, -0.5), (0.24, 0.12, 0.1)], name='shoulder')),
        Sh(Ba([(0.12, -0.06, -0.08), (0.47, 0.06, 0.08)], name='upperArm')),
        None,
        Sh(Ba([(0.07, -0.06, -0.055), (0.18, 0.06, 0.03)], name='foreArm1'),
           Ba([(0.18, -0.06, -0.03), (0.36, 0.06, 0.03)], name='foreArm2')).applyTrans(pose),
        None, 
        None,
        None]
    return links

params = {'fingerLength' : 0.06,
          'fingerWidth' :  0.04,
          'fingerThick' :  0.02,
          'palmLength' : 0.09,
          'palmWidth' : 0.175,
          'palmThick' : 0.05,
          'gripperOffset': 0.04,
          'gripMax' :      0.081,
          'zRange': (0.0, 1.5),
          'zRangeHand': (0.35, 2.0),
          }

def pr2GripperLinks():
    palm_dx = params['palmLength']
    palm_dy = params['palmWidth']
    palm_dz = params['palmThick']
    fing_dx = params['fingerLength']
    fing_dy = params['fingerWidth']
    fing_dz = params['fingerThick']
    return [Sh(shapes.Box(palm_dx, palm_dy, palm_dz, Ident, name='palm')),
            Sh(shapes.Box(fing_dx, fing_dy, fing_dz, Ident, name='finger1')),
            Sh(shapes.Box(fing_dx, fing_dy, fing_dz, Ident, name='finger2'))]

def pr2GripperJoints(arm):
    o = params['gripperOffset']
    palm_dx = params['palmLength']
    fing_dx = params['fingerLength']
    fing_dy = params['fingerWidth']
    fing_dz = params['fingerThick']
    return [Rigid(arm+'_palm',
                  (r_forceSensorOffset if arm == 'r' else l_forceSensorOffset) * \
                  hu.Transform(transf.translation_matrix([o + palm_dx/2.,0.0,0.0])),
                  None, None),
            Prismatic(arm+'_finger1',
                      hu.Transform(transf.translation_matrix([palm_dx/2+fing_dx/2.,fing_dy/2.,0.0])),
                      (0.0, params['gripMax']/2.), (0.,1.,0)),
            Prismatic(arm+'_finger2',
                      hu.Transform(transf.translation_matrix([0.0,-fing_dy,0.0])),
                      (0.0, 0.081), (0.,-1.,0))]

# Transforms from wrist frame to hand frame
# This leaves X axis unchanged
left_gripperToolOffsetX = hu.Pose(0.18,0.0,0.0,0.0)
right_gripperToolOffsetX = hu.Transform(np.dot(r_forceSensorOffset.matrix,
                                                 left_gripperToolOffsetX.matrix))                          
# This rotates around the Y axis... so Z' points along X an X' points along -Z
left_gripperToolOffsetZ = hu.Transform(np.dot(left_gripperToolOffsetX.matrix,
                                               transf.rotation_matrix(math.pi/2,(0,1,0))))
right_gripperToolOffsetZ = hu.Transform(np.dot(right_gripperToolOffsetX.matrix,
                                                 transf.rotation_matrix(math.pi/2,(0,1,0))))

# Rotates wrist to grasp face frame
gFaceFrame = hu.Transform(np.array([(0.,1.,0.,0.18),
                                      (0.,0.,1.,0.),
                                      (1.,0.,0.,0.),
                                      (0.,0.,0.,1.)]))

gripperFaceFrame = {'left': gFaceFrame, 'right': r_forceSensorOffset*gFaceFrame}

################################################################
# Conf and CartConf, mostly generic
################################################################

# This behaves like a dictionary, except that it doesn't support side effects.
class JointConf(RobotJointConf):
    def __init__(self, conf, robot):
        RobotJointConf.__init__(self, conf, robot)
    def copy(self):
        return JointConf(self.conf.copy(), self.robot)
    def handWorkspace(self):
        tz = self.conf['pr2Torso'][0]
        bb = ((0.5, -0.25, tz+0.2),(0.75, 0.25, tz+0.4)) # low z, so view cone extends
        return shapes.BoxAligned(np.array(bb), Ident).applyTrans(self.basePose())

class CartConf(RobotCartConf):
    def __init__(self, conf, robot):
        RobotCartConf.__init__(self, conf, robot)
    def copy(self):
        return CartConf(self.conf.copy(), self.robot)

# Stow angles
rightStowAngles = [-2.1, 1.29, 0.000, -0.15, 0.000, -0.100, 0.000]
leftStowAngles = [2.1, 1.29, 0.000, -0.15, 0.000, -0.100, 0.000]
# Tuck angles
rightTuckAngles = [-1.1432, 1.1099, -1.4, -2.1064, -1.3347, -1.1165, 2.1908]
leftTuckAngles = [1.1432, 1.1099, 1.4, -2.1064, 1.3347, -1.1165, -2.1908]
# This is a joint configuartion, specifying the joint angles for all the chains.
pr2Init = {'pr2Base':[0.0,0.0,0.0],
           'pr2Torso':[TORSO_Z],
           'pr2LeftArm': leftStowAngles,
           'pr2LeftGripper': [0.02],
           'pr2RightArm': rightStowAngles,
           'pr2RightGripper': [0.02],
           'pr2Head': [0.0, 0.0]}

# In a cartesian configuration, we specify frames for base, left and right
# hands, and head.

################################################################
# This is used to make the robot instances
################################################################

pr2Chains = {}
def makePr2Chains(name, workspaceBounds, new=True):
    global pr2Chains
    if not new and name in pr2Chains:
         return pr2Chains[name]
    # Chain for base
    baseChain = Planar('pr2Base', 'root', pr2BaseLink, workspaceBounds)
    # Chain for torso
    torsoChain = Chain('pr2Torso', 'pr2Base_theta',
                       getUrdfJoints(pr2_torso_joints, urdfPath),
                       pr2TorsoLinks)
    # Chain for left arm
    leftArmChain = Chain('pr2LeftArm', 'torso_lift_joint',
                         getUrdfJoints(armJointNames('l', pr2_arm_joints),
                                           urdfPath),
                         pr2ArmLinks('l'))
    # Chain for left gripper
    leftGripperChain = GripperChain('pr2LeftGripper', 'l_wrist_roll_joint',
                                    pr2GripperJoints('l'),
                                    pr2GripperLinks())
    # Chain for right arm
    rightArmChain = Chain('pr2RightArm', 'torso_lift_joint',
                         getUrdfJoints(armJointNames('r', pr2_arm_joints),
                                           urdfPath),
                         pr2ArmLinks('r'))
    # Chain for right gripper
    rightGripperChain = GripperChain('pr2RightGripper', 'r_wrist_roll_joint',
                                     pr2GripperJoints('r'),
                                     pr2GripperLinks())
    # Chain for head
    headChain = Chain('pr2Head', 'torso_lift_joint',
                      getUrdfJoints(pr2_head_joints, urdfPath),
                      pr2HeadLinks)
    pr2Chains[name] = MultiChain(name,
                           [baseChain, torsoChain, leftArmChain, leftGripperChain,
                            rightArmChain, rightGripperChain, headChain])
    return pr2Chains[name]

# The radius is baseCovariance radius, angle in baseCovariance angle,
# reachPct is percentage of maximum reach of arm.
def makePr2ChainsShadow(name, workspaceBounds, radiusVar=0.0, angleVar=0.0, reachPct=1.0):
    sqrt2 = 2.0**0.5
    gr = radiusVar + sqrt2*0.33*angleVar
    pr2BaseLinkGrown = Sh(\
        Ba([(-0.43-gr, -0.33-gr, 0.0), (0.33+gr, 0.33+gr, 0.33)], name='baseGrown'),
        Ba([(-0.43-gr, -0.33-gr, 0.0), (0.1, 0.33+gr, 1.0)], name='torsoGrown')
        )
    def pr2ArmLinksGrown(arm):
        angle = math.pi/2 if arm=='r' else -math.pi/2
        pose = hu.Transform(transf.rotation_matrix(angle, (1,0,0)))
        gr1 = radiusVar + 0.43*reachPct*angleVar
        gr2 = radiusVar + 0.76*reachPct*angleVar
        gr3 = radiusVar + 1.05*reachPct*angleVar
        #print 'gr1', gr1, 'gr2', gr2, 'gr3', gr3
        # raw_input('arm growth factors')
        links = [\
            Sh(Ba([(-0.10-gr1, -0.12-gr1, -0.5), (0.24+gr1, 0.12+gr1, 0.1)], name='shoulder')),
            Sh(Ba([(0.12-gr2, -0.06-gr2, -0.08-gr2), (0.47+gr2, 0.06+gr2, 0.08+gr2)], name='upperArm')),
            None,
            Sh(Ba([(0.07-gr3, -0.06-gr3, -0.055-gr3), (0.18+gr3, 0.06+gr3, 0.03+gr3)], name='foreArm1'),
               Ba([(0.18-gr3, -0.06-gr3, -0.03-gr3), (0.36+gr3, 0.06+gr3, 0.03+gr3)], name='foreArm2')).applyTrans(pose),
            None, 
            None,
            None]
        return links
    def pr2GripperLinksGrown():
        palm_dx = params['palmLength']
        palm_dy = params['palmWidth']
        palm_dz = params['palmThick']
        fing_dx = params['fingerLength']
        fing_dy = params['fingerWidth']
        fing_dz = params['fingerThick']
        gr = radiusVar + 1.2*reachPct*angleVar
        return [Sh(shapes.Box(palm_dx+2*gr, palm_dy+2*gr, palm_dz+2*gr, Ident, name='palm')),
                Sh(shapes.Box(fing_dx+2*gr, fing_dy+2*gr, fing_dz+2*gr, Ident, name='finger1')),
                Sh(shapes.Box(fing_dx+2*gr, fing_dy+2*gr, fing_dz+2*gr, Ident, name='finger2'))]

    # Chain for base
    baseChain = Planar('pr2Base', 'root', pr2BaseLinkGrown, workspaceBounds)
    # Chain for torso
    torsoChain = Chain('pr2Torso', 'pr2Base_theta',
                       getUrdfJoints(pr2_torso_joints, urdfPath),
                       pr2TorsoLinks)     # unchanged
    # Chain for left arm
    leftArmChain = Chain('pr2LeftArm', 'torso_lift_joint',
                         getUrdfJoints(armJointNames('l', pr2_arm_joints), urdfPath),
                         pr2ArmLinksGrown('l'))
    # Chain for left gripper
    leftGripperChain = GripperChain('pr2LeftGripper', 'l_wrist_roll_joint',
                                    pr2GripperJoints('l'),
                                    pr2GripperLinks()) # NB
    # Chain for right arm
    rightArmChain = Chain('pr2RightArm', 'torso_lift_joint',
                         getUrdfJoints(armJointNames('r', pr2_arm_joints), urdfPath),
                         pr2ArmLinksGrown('r'))
    # Chain for right gripper
    rightGripperChain = GripperChain('pr2RightGripper', 'r_wrist_roll_joint',
                                     pr2GripperJoints('r'),
                                     pr2GripperLinks()) # NB
    # Chain for head
    headChain = Chain('pr2Head', 'torso_lift_joint',
                      getUrdfJoints(pr2_head_joints, urdfPath),
                      pr2HeadLinks)       # unchanged
    return MultiChain(name,
                      [baseChain, torsoChain, leftArmChain, leftGripperChain,
                       rightArmChain, rightGripperChain, headChain])

################################################################
# The actual robot class, a subclass of GenericRobot
################################################################

# These don't handle the rotation correctly -- what's the general form ??
def fliph(pose):
    params = list(pose.pose().xyztTuple())
    params[1] = -params[1]
    return hu.Pose(*params)

def flipv(pose):
    m = pose.matrix.copy()
    m[1,3] = -m[1,3]
    return hu.Transform(m)

def ground(pose):
    params = list(pose.xyztTuple())
    params[2] = 0.0
    return hu.Pose(*params)

# Does not include the torso
pr2ChainDependencies = \
                     {'pr2Base' : ['pr2Base', 'pr2Head',
                                   'pr2LeftArm', 'pr2LeftGripper', 'pr2RightGripper', 'pr2RightArm',
                                   ],
                      'pr2Torso' : ['pr2Torso', 'pr2Head',
                                   'pr2LeftArm', 'pr2LeftGripper', 'pr2RightGripper', 'pr2RightArm',
                                   ],
                      'pr2Head' : ['pr2Head'],
                      'pr2LeftArm' : ['pr2LeftArm', 'pr2LeftGripper',
                                      ],
                      'pr2LeftGripper' : ['pr2LeftGripper',
                                          ],
                      'pr2RightArm' : ['pr2RightArm', 'pr2RightGripper',
                                       ],
                      'pr2RightGripper' : ['pr2RightGripper',
                                           ]
                      }

pr2ChainDependenciesRev = \
                        {'pr2Base' : [],
                         'pr2Torso' : ['pr2Base'],
                         'pr2Head' : ['pr2Base'],
                         'pr2LeftArm' : ['pr2Base'],
                         'pr2LeftGripper' : ['pr2LeftArm', 'pr2Base'],
                         'pr2RightArm' : ['pr2Base'],
                         'pr2RightGripper' : ['pr2RightArm', 'pr2Base']
                      }

# This basically implements a Chain type interface, execpt for the wstate
# arguments to the methods.
class PR2(GenericRobot):
    def __init__(self, name, chains, color = pr2Color, useLeft=True, useRight=True):
        self.chains = chains
        self.color = color
        self.name = name
        self.useLeft = useLeft
        self.useRight = useRight
        # These are (focal, height, width, length, n)
        self.scanner = (0.3, 0.2, 0.2, 5, 30) # Kinect
        # These names encode the "order of actuation" used in interpolation
        self.chainNames = ['pr2LeftGripper', 'pr2RightGripper',
                           'pr2Torso', 'pr2Base',
                           'pr2LeftArm', 'pr2RightArm', 'pr2Head']
        self.bodyChains = ['pr2Torso', 'pr2Base', 'pr2Head']
        # This is overriden by testRig.py
        self.moveChainNames = ['pr2Base']
        if self.useRight: self.moveChainNames.append('pr2RightArm')
        if self.useLeft: self.moveChainNames.append('pr2LeftArm')
        self.armChainNames = {'left':'pr2LeftArm', 'right':'pr2RightArm'}
        self.gripperChainNames = {'left':'pr2LeftGripper', 'right':'pr2RightGripper'}
        self.wristFrameNames = {'left':'l_wrist_roll_joint', 'right':'r_wrist_roll_joint'}
        self.baseChainName = 'pr2Base'
        self.headChainName = 'pr2Head'
        self.selfCollideChainNames = [[self.armChainNames['left']], [self.gripperChainNames['left']],
                                      [self.armChainNames['right']], [self.gripperChainNames['right']],
                                      self.bodyChains]
        # This has the X axis pointing along fingers
        self.toolOffsetX = {'left': left_gripperToolOffsetX, 'right': right_gripperToolOffsetX}
        # This has the Z axis pointing along fingers (more traditional, as in ikFast)
        self.toolOffsetZ = {'left': left_gripperToolOffsetZ, 'right': right_gripperToolOffsetZ}
        self.gripperFaceFrame = gripperFaceFrame
        self.gripMax = params['gripMax']
        self.nominalConf = None
        self.armStowAngles = {'right': rightStowAngles, 'left': leftStowAngles}
        self.armTuckAngles = {'right': rightTuckAngles, 'left': leftTuckAngles}

        if USE_OLD_IK_POSES:
            # Using robot.pr2.pr2IkPoses
            horizontalTrans, verticalTrans = ikTrans(level=2) # include more horizontal confs
            self.horizontalTrans = {'left': [p.inverse() for p in horizontalTrans],
                                    'right': [fliph(p).inverse() for p in horizontalTrans]}
            self.verticalTrans = {'left': [p.inverse() for p in verticalTrans],
                                  'right': [flipv(p).inverse() for p in verticalTrans]}
        else:
            self.horizontalTrans = {}
            self.verticalTrans = {}
            ikData = ikTransLRz()
            for hand in ('left', 'right'):
                self.horizontalTrans[hand] = [(z, e.inverse()) for (c,z, e) in ikData[hand] if c == 'h']
                self.verticalTrans[hand] = [(z, e.inverse()) for (c,z,e) in ikData[hand] if c == 'v']

        self.confCache = {}
        self.confCacheKeys = deque([])  # in order of arrival

        self.compiledChainsOS = compileChainFramesOS(self)
        self.chainDependencies = pr2ChainDependencies
        self.chainDependRev = pr2ChainDependenciesRev
        self.OSa = None

    def handNames(self):
        hands = []
        if self.useRight: hands.append('right')
        if self.useLeft: hands.append('left')
        return hands

    def makeJointConf(self, confDict):
        return JointConf(confDict, self)

    def makeCartConf(self, conf):
        return CartConf(conf, self)

    def clearInvKinCache(self):
        clearInvKinCache2()

    # This should not be used outside of this file...
    # The base transforms take into account any twist in the tool offset
    def potentialBasePosesGen(self, wrist, hand, n=None, complain=True):
        gripper = wrist*(left_gripperToolOffsetX if hand=='left' else right_gripperToolOffsetX)
        xAxisZ = gripper.matrix[2,0]
        if abs(xAxisZ) < 0.5:
            trs = self.horizontalTrans[hand]
        elif abs(xAxisZ + 1.0) < 0.5:
            trs = self.verticalTrans[hand]
        else:
            if complain:
                print 'gripper=\n', gripper.matrix
                raw_input('Illegal gripper trans for base pose')
            return
        for i, data in enumerate(trs):
            if USE_OLD_IK_POSES:
                tr = data
            else:
                (z, tr) = data          # z is nominal wrist z
                if abs(z - wrist.matrix[2,3]) > 0.1: # if too far, then ignore
                    continue
            if n and i > n: return
            # use largish zthr to compensate for twist in force sensor
            ans = wrist.compose(tr).pose(zthr = 0.1, fail=False) 
            if ans is None:
                if complain:
                    print 'gripper=\n', gripper.matrix
                    raw_input('Illegal gripper trans for base pose')
                return
            yield ground(ans)

    def fingerSupportFrame(self, hand, width):
        # The old way...
        # Origin is on the inside surface of the finger (at the far tip).
        # The -0.18 is from finger tip to the wrist  -- if using wrist frame
        # mat = np.dot(transf.euler_matrix(-math.pi/2, math.pi/2, 0.0, 'ryxz'),
        #              transf.translation_matrix([0.0, -0.18, -width/2]))

        # y points along finger approach, z points in closing direction
        # offset aligns with the grasp face.
        # This is global gripperFaceFrame offset to center object
        gripperFaceFrame_dy = hu.Transform(np.array([(0.,1.,0.,0.18),
                                                       (0.,0.,1.,-width/2),
                                                       (1.,0.,0.,0.),
                                                       (0.,0.,0.,1.)]))
        if hand == 'right':
            gripperFaceFrame_dy = r_forceSensorOffset * gripperFaceFrame_dy
        return gripperFaceFrame_dy

    # This is useful to get the base shape when we don't yet have a conf
    def baseLinkShape(self, basePose=None):
        if basePose:
            return pr2BaseLink.applyTrans(basePose)
        else:
            return pr2BaseLink

    # This is a "partial" inverse kinematics involving only one hand
    # and (optionally) a basePose and/or grip.  The defaultConf
    # specifies the other parameters.  In the missing basePose case,
    # it should generate alternatives.
    def inverseKinWristGen(self, wrist, hand, defaultConf,
                             basePose=None, grip=None,
                             complain=True, n=50, counts=None):
        if basePose:                    # base specified
            conf = self.confFromBaseAndWrist(basePose, hand, wrist,
                                             defaultConf, counts=None, grip=grip)
            if conf and all(conf.conf.values()):
                yield conf
            return                      # there is only one...
        else:
            for basePose in \
                    self.potentialBasePosesGen(wrist, hand,
                                               n=n, complain=complain):
                conf = self.confFromBaseAndWrist(basePose, hand, wrist,
                                                 defaultConf, counts=counts, grip=grip)
                if conf and all(conf.conf.values()):
                    yield conf

    def confFromBaseAndWrist(self, basePose, hand, wrist,
                             defaultConf, counts=None, grip=params['gripMax'],
                             freeBase=False):
        ans = self.confFromBaseAndWristAux(basePose, hand, wrist,
                                           defaultConf, counts=counts, grip=grip)
        if ans: return ans
        # Not sure this is useful... This is where optimization would be useful...
        if freeBase:
            count = 0
            (x0,y0,z,t0) = basePose.pose().xyztTuple()
            for x in (x0, x0-0.01, x0+0.01):
                for y in (y0, y0-0.01, y0+0.01):
                    for t in (t0, t0-0.02, t0+0.02):
                        bp = hu.Pose(x,y,z,t)
                        ans = self.confFromBaseAndWristAux(bp, hand, wrist,
                                                      defaultConf, counts=counts, grip=grip)
                        if ans:
                            if count > 0: print 'invKin recovery in freeBase', count
                            return ans
                        count += 1
            print 'Tried', count, 'invKins during freeBase'

    def confFromBaseAndWristAux(self, basePose, hand, wrist,
                                defaultConf, counts=None, grip=None):
        robot = defaultConf.robot
        # Get a grip...
        grip = grip or defaultConf[robot.gripperChainNames[hand]]
        grip = min(params['gripMax']-0.001, grip) # don't exceed the max
        # Make a cartConf
        cart = CartConf({'pr2BaseFrame': basePose,
                         'pr2Torso':[TORSO_Z]}, self)
        if hand == 'left':
            cart.conf['pr2LeftArmFrame'] = wrist 
            cart.conf['pr2LeftGripper'] = [grip] # !! pick better value
        else:
            cart.conf['pr2RightArmFrame'] = wrist 
            cart.conf['pr2RightGripper'] = [grip]
        # Check inverse kinematics
        conf = self.inverseKin(cart, defaultConf)
        if None in conf.values():
            if counts: counts[0] += 1       # kin failure
            return
        assert all(c in conf.conf for c in ['pr2RightArm', 'pr2LeftArm',
                                            'pr2RightGripper', 'pr2LeftGripper',
                                            'pr2Head', 'pr2Base'])
        # Copy the other arm from pbs
        # if hand == 'left':
        #     conf.conf['pr2RightArm'] = defaultConf['pr2RightArm']
        #     conf.conf['pr2RightGripper'] = [grip]
        # else:
        #     conf.conf['pr2LeftArm'] = defaultConf['pr2LeftArm']
        #     conf.conf['pr2LeftGripper'] = [grip]
        return conf

    # Note that the "ArmFrame" is the wrist frame.
    def forwardKin(self, conf, complain = False, fail = False):
        shapes, frames = self.placement(conf, getShapes=[])
        return CartConf(\
            {'pr2BaseFrame': frames[self.chains.chainsByName['pr2Base'].joints[-1].name],
             'pr2LeftArmFrame':
             frames[self.chains.chainsByName['pr2LeftArm'].joints[-1].name],
             'pr2RightArmFrame':
             frames[self.chains.chainsByName['pr2RightArm'].joints[-1].name],
             'pr2HeadFrame': frames[self.chains.chainsByName['pr2Head'].joints[-1].name],
             'pr2LeftGripper': conf['pr2LeftGripper'],
             'pr2RightGripper': conf['pr2RightGripper'],
             'pr2Torso': conf['pr2Torso']},
            self)

    # Only some chains are usually present in cart, other ones are
    # taken from conf.
    def inverseKin(self, cart, conf,
                   returnAll = False, complain = False, fail = False):
        """Map from cartesian configuration (wrists and head) to joint
        configuration."""
        assert conf or self.nominalConf, 'inverseKin needs reference conf'
        conf = conf or self.nominalConf
        # Base
        if 'pr2BaseFrame' in cart.conf:
            baseTrans = cart.get('pr2Base', None).pose()
            # Optimize this...
            # baseChain = self.chains.chainsByName['pr2Base']
            # baseAngles = baseChain.inverseKin(Ident, baseTrans)
            baseAngles = baseTrans.xytTuple()
            if not baseAngles:
                if complain: print 'Failed invkin for base'
                if fail: raise Exception, 'Failed invkin for base'
                conf = conf.set('pr2Base', None) # explicitly show failure
            else:
                conf = conf.set('pr2Base', list(baseAngles))
        else:
            # Keep what is in conf
            baseTrans = conf.basePose()
        # First, pick a torso value, since that affects hands and head.
        if 'pr2Torso' in cart.conf:
            torsoZ = cart['pr2Torso'][0]
        else:
            torsoZ = torsoInvKin(self.chains, baseTrans,
                                 cart.get('pr2LeftArm', cart.get('pr2RightArm')))
        conf = conf.set('pr2Torso', [torsoZ])
        torsoTrans = self.chains.chainsByName['pr2Torso'].forwardKin(baseTrans, [torsoZ])
        # Solve for arms
        if 'pr2LeftArmFrame' in cart.conf:
            leftArmAngles = armInvKin2(self.chains,
                                       'l', torsoTrans,
                                       cart['pr2LeftArm'],
                                       # if a nominal conf is available use as reference
                                       conf or self.nominalConf,
                                       returnAll = returnAll)
            if not leftArmAngles:
                if complain:
                    raw_input('Failed invkin for left arm')
                if fail: raise Exception, 'Failed invkin for left arm'
                conf = conf.set('pr2LeftArm', None)
            else:
                conf = conf.set('pr2LeftArm', leftArmAngles)
        if 'pr2RightArmFrame' in cart.conf:
            rightArmAngles = armInvKin2(self.chains,
                                         'r', torsoTrans,
                                         cart['pr2RightArm'],
                                         # if a nominal conf is available use as reference
                                         conf or self.nominalConf,
                                         returnAll = returnAll)
            if not rightArmAngles:
                if complain:
                    raw_input('Failed invkin for right arm')
                if fail: raise Exception, 'Failed invkin for right arm'
                conf = conf.set('pr2RightArm', None)
            else:
                conf = conf.set('pr2RightArm', rightArmAngles)
        if 'pr2HeadFrame' in cart.conf:
            headAngles = headInvKin(self.chains,
                                    torsoTrans, cart['pr2Head'])
            if not headAngles:
                if complain: print 'Failed invkin for head'
                if fail: raise Exception, 'Failed invkin for head'
                conf = conf.set('pr2Head', None)
            else:
                conf = conf.set('pr2Head', headAngles)
        if 'pr2LeftGripper' in cart.conf:
            g = cart.conf['pr2LeftGripper']
            conf = conf.set('pr2LeftGripper', g if isinstance(g, (list,tuple)) else [g])
        if 'pr2RightGripper' in cart.conf:
            g = cart.conf['pr2RightGripper']
            conf = conf.set('pr2RightGripper', g if isinstance(g, (list,tuple)) else [g])
        return conf

    def makeConf(self,x,y,th,g=0.07, vertical=False, left=None, right=None,
                 ignoreStandard=False):
        global standardVerticalConf, standardHorizontalConf
        if left is None: left = self.useLeft
        if right is None: right = self.useRight
        dx = dy = dz = 0
        dt = 0.0 if vertical else TORSO_Z - 0.3
        if vertical and standardVerticalConf and (not ignoreStandard):
            c = standardVerticalConf.copy()
            c.conf['pr2Base'] = [x, y, th]
            if self.useLeft:
                c.conf['pr2LeftGripper'] = [g]
            if self.useRight:
                c.conf['pr2RightGripper'] = [g]
            return c
        elif (not vertical) and standardHorizontalConf and (not ignoreStandard):
            c = standardHorizontalConf.copy()
            c.conf['pr2Base'] = [x, y, th]
            if self.useLeft:
                c.conf['pr2LeftGripper'] = [g]
            if self.useRight:
                c.conf['pr2RightGripper'] = [g]
            return c
        else:
            c = JointConf(pr2Init.copy(), self)
            c = c.set('pr2Base', [x, y, th])
            if self.useLeft:
                c = c.set('pr2LeftGripper', [g])
            if self.useRight:
                c = c.set('pr2RightGripper', [g])
            cart = c.cartConf()
            base = cart['pr2Base']
            if vertical:
                q = np.array([0.0, 0.7071067811865475, 0.0, 0.7071067811865475])
                if left and not right:
                    h = hu.Transform(p=np.array([[a] for a in [ 0.4+dx, 0.2+dy,  1.1+dt, 1.]]), q=q)
                    cart = cart.set('pr2LeftArm', base.compose(h))
                else:
                    cart = cart.rem('pr2LeftArm')
                if right:
                    hr = hu.Transform(p=np.array([[a] for a in [ 0.4+dx, -(0.2+dy),  1.1+dt, 1.]]), q=q)
                    cart = cart.set('pr2RightArm', base.compose(hr))
            else:
                if left and not right:
                    h = hu.Pose(0.3+dx,0.3+dy,0.9+dz+dt,-math.pi/2)
                    cart = cart.set('pr2LeftArm', base.compose(h))
                else:
                    cart = cart.rem('pr2LeftArm')
                if right:
                    hr = hu.Pose(0.3+dx,-(0.3+dy),0.9+dz+dt,math.pi/2)
                    cart = cart.set('pr2RightArm', base.compose(hr))
            c = self.inverseKin(cart, c)

            c.conf['pr2Head'] = [0., 0.]
            assert all(c.values())
            if vertical:
                standardVerticalConf = c
            else:
                standardHorizontalConf = c
            return c

    def handViewPoints(self):
        return [hu.Point(np.array([[0.5], [0.0], [1.0], [1.]])),
                hu.Point(np.array([[0.5], [0.0], [0.8], [1.]]))]

########################################
# Inverse kinematics support functions
########################################

headInvKinCacheStats = [0,0]
headInvKinCache = {}

def headInvKin(chains, torso, targetFrame, allowedViewError = 1e-3):
    headChain = chains.chainsByName['pr2Head']
    limits = headChain.limits()
    # Displacement from movable joints to sensor
    headSensorOffsetY = reduce(np.dot, [j.trans for j in headChain.joints[2:-1]]).matrix
    headSensorOffsetZ = reduce(np.dot, [j.trans for j in headChain.joints[1:-1]]).matrix

    headRotationFrameZ = np.dot(torso, headChain.joints[0].trans)
    # Target point relative to torso
    relFramePoint = torso.inverse().compose(targetFrame).point()

    key = relFramePoint
    headInvKinCacheStats[0] += 1
    if key in headInvKinCache:
        headInvKinCacheStats[1] += 1
        return headInvKinCache[key]

    targetZ = headRotationFrameZ.inverse().applyToPoint(targetFrame.point()).matrix
    angles1 = tangentSol(targetZ[0,0], targetZ[1,0], headSensorOffsetZ[0,3], headSensorOffsetZ[1,3])    
    angles1 = set(angles1 + list(limits[0]))
    
    best = None
    bestScore = 1.0e10
    bestError = None
    # print 'zero\n', headChain.forwardKin(torso, (0, 0)).matrix
    for a1 in angles1:
        headRotationFrameZ = np.dot(torso, headChain.joints[0].transform(a1))
        headRotationFrameY = np.dot(headRotationFrameZ, headChain.joints[1].trans)
        targetY = headRotationFrameY.inverse().applyToPoint(targetFrame.point()).matrix
        angles2 = [-x for x in tangentSol(targetY[0,0], targetY[2,0],
                                          headSensorOffsetY[0,3], headSensorOffsetY[2,3])]
        angles2 = set(angles2 + list(limits[1]))

        for a2 in angles2:
            if headChain.valid([a1, a2]):
                headTrans = headChain.forwardKin(torso, (a1, a2))
                sensorCoords = headTrans.inverse().applyToPoint(targetFrame.point()).matrix
                if sensorCoords[2] < 0.: continue
                score = math.sqrt(sensorCoords[0]**2 + sensorCoords[1]**2)
                if score < bestScore:
                    best = [a1, a2]
                    bestScore = score

    ans = best if bestScore <= allowedViewError else None
    headInvKinCache[key] = ans
    return ans

def tangentSolOld(x, y, x0, y0):
    # print 'X', (x,y), 'X0', (x0, y0)
    alpha = math.atan2(y,x)
    theta0 = math.atan2(y0,x0)
    r = math.sqrt(x0*x0 + y0*y0)
    d = math.sqrt(x*x + y*y)
    theta1 = math.pi/2
    ac = math.acos((r/d)*math.cos(theta0-theta1))
    # print 'alpha', alpha, 'theta0', theta0, 'ac', ac
    values =  [alpha + theta1 + ac,
               alpha + theta1 - ac,
               alpha - theta1 + ac,
               alpha - theta1 - ac]
    keep = []
    for angle in values:
        v = (x - r*math.cos(angle + theta0),
             y - r*math.sin(angle + theta0))
        vangle = math.atan2(v[1], v[0])
        # print 'angle', angle, 'atan', vangle
        if hu.angleDiff(angle, vangle) < 0.001:
            keep.append(vangle)
    # This generally returns two answers, but they may not be within limits.
    # print 'keep', keep
    return keep

# x0,y0 is sensor point (rotation is at origin)
# x,y is target point
# The sensor is pointing along the x axis when angle is zero!
# At desired sensor location we have a triangle (origin, sensor, target)
# l is distance from sensor to target, found from law of cosines.
# alpha is angle target-origin-sensor, also from law of cosines
def tangentSol(x, y, x0, y0):
    def quad(a,b,c):
        disc = b*b - 4*a*c
        if disc < 0: return []
        discr = disc**0.5
        return [(-b + discr)/(2*a), (-b - discr)/(2*a)]
        
    ph= math.atan2(y,x)
    d = math.sqrt(x*x + y*y)
    th0 = math.atan2(y0,x0)
    r = math.sqrt(x0*x0 + y0*y0)
    # c1 is cos of angle between sensor direction and line to x0,y0
    c1 = math.cos(math.pi - th0)
    # vals for l
    lvals = quad(1.0, -2*r*c1, r*r-d*d)
    # print 'lvals', lvals
    if not lvals: return []
    # vals for alpha
    avals = [math.acos(max(-1.0, min(1.0, (l*l - r*r - d*d)/(-2*r*d)))) for l in lvals]
    # print 'avals', avals
    # angs are candidate rotation angles
    angs = []
    for alpha in avals:
        angs.extend([ph - alpha - th0, ph + alpha - th0])
    angs = [hu.fixAnglePlusMinusPi(a) for a in angs]
    # print 'angs', angs
    ans = []
    for ang in angs:
        # check each angle, (x1,y1) is sensor location
        x1 = r*math.cos(th0 + ang)
        y1 = r*math.sin(th0 + ang)
        # distance sensor-target
        l = math.sqrt((x1-x)**2 + (y1-y)**2)
        # the sensor direction rotates by ang, check that it points at target
        ex = abs(x - (x1 + l*math.cos(ang)))
        ey = abs(y - (y1 + l*math.sin(ang)))
        # print 'ang', ang, 'ex', ex, 'ey', ey
        # keep the ones with low error
        if ex < 0.001 and ey < 0.001:
            ans.append(ang)
    return ans

def torsoInvKin(chains, base, target):
    # Should pick a good torso value to place the hand at target, prefering
    # not to change the current value if possible.
    return TORSO_Z

################################################################
# Interface for testRig to create robot instances
################################################################

def makeRobot(workspace, useLeft=True, useRight=True, radiusVar=0.0):
    robot = PR2('MM', makePr2ChainsShadow('PR2', workspace, radiusVar=radiusVar),
                useLeft=useLeft, useRight=useRight)
    # This affects randomConf and stepAlongLine, unless overriden
    robot.moveChainNames = ['pr2Base']
    if useLeft:
        robot.moveChainNames.extend(['pr2LeftArm', 'pr2LeftGripper'])
    if useRight:
        robot.moveChainNames.extend(['pr2RightArm', 'pr2RightGripper'])
    # This uses the initial Conf as a reference for inverseKin, when no conf is given
    robot.nominalConf = JointConf(pr2Init, robot)
    return robot
