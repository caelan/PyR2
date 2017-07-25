import math
import os

import numpy as np
import geometry.hu as hu

from geometry.transformations import euler_matrix, euler_from_matrix, quaternion_from_matrix

curDir = os.path.dirname(os.path.abspath(__file__))
ikPath = curDir + '/pr2IKData.py'

# Generate a (dense) sample of robot arm angles that correspond to
# either horizontal or vertical grasps.  Each hand placement defines a
# candidate base position to achieve a grasp.

# Each grasp is characterized by Z value and a score that reflects
# desirability.  We'll keep the grasps (arm angles) sorted by these
# values.  We'll be able to pick out relevant arm confs by slicing on
# the Z values.  

def writeIKData(robot, angleStep = math.pi/20.):
    f = open(ikPath, 'w')
    try:
        f.write('leftArmIKData = ')
        leftScan = kinScan(robot, 'left', angleStep)
        f.write(str(leftScan)); f.write('\n\n')
        # Could try to exploit the symmetry to avoid a second scan
        f.write('rightArmIKData = ')
        rightScan = kinScan(robot, 'right', angleStep)
        f.write(str(rightScan)); f.write('\n\n')
    finally:
        f.close()

def readIKData():
    from geometry.pr2.pr2IKData import rightArmIKData, leftArmIKData
    return {'right': rightArmIKData, 'left': leftArmIKData}

def ikTransLR():
    def process(data):
        # (grasp class, trans)
        return [(e[0][0], transFromQPRep(e[1][0])) for e in data]
    ikData = readIKData()
    return {h:process(data) for (h,data) in ikData.items()}

def ikTransLRz():
    def process(data):
        # (grasp class, z, trans)
        return [(e[0][0], e[0][1], transFromQPRep(e[1][0])) for e in data]
    ikData = readIKData()
    return {h:process(data) for (h,data) in ikData.items()}

def kinScan(robot, hand, angleStep):
    armChainName = robot.armChainNames[hand]
    limits = list(robot.limits([armChainName])) # an iterable of joint limits
    initConf = robot.makeConf(0,0,0)
    limEps = math.pi/20
    def kinScanAux(vals, lims):
        if lims:
            thlim = lims[0]
            confs = []
            fullRange = (thlim[1] - thlim[0] > 2*math.pi-0.001)
            # Scan the range but stay away from the actual limits
            for th in np.arange(thlim[0] if fullRange else thlim[0]+limEps,
                                thlim[1]-limEps,
                                angleStep):
                if not vals: print th
                confs.extend(kinScanAux(vals+[th], lims[1:]))
            return confs
        else:
            # either [] or [entry]
            return kinScanEntry(initConf, hand, vals)
    kinVals = kinScanAux([], limits)
    kinVals.sort()                      # each entry is ((class,z,score), (wrist, angles))
    return kinVals

def kinScanEntry(conf, hand, angles):
    armChainName = conf.robot.armChainNames[hand]
    c = conf.set(armChainName, angles)
    cart = c.cartConf()
    gclass, euler = graspClass(cart[armChainName])
    if not gclass: return []
    nangles = refineAngles(c, cart, armChainName, euler)
    if not nangles: return []
    # get wrist for nangles
    wrist = conf.set(armChainName, nangles).cartConf()[armChainName]
    return [((gclass, wrist.matrix[2,3], kinScore(gclass, wrist)),
             (qpRep(wrist), nangles))]

# Write out transforms as quaternion, position tuples of lists.
def qpRep(tr):
    return (quaternion_from_matrix(tr.matrix).tolist(),
            tr.matrix[:,3].tolist())

def transFromQPRep(qp):
    (q, p) = qp
    return hu.Transform(p=np.array([[a] for a in p]), q=np.array(q))

vwrist = hu.Transform(np.array([[0.0, 0.0, 1.0, 1.10],
                                [0.0, 1.0, 0.0, 0.0],
                                [-1.0, 0.0, 0.0, 1.0],
                                [0.0, 0.0, 0.0, 1.0]]))

def kinScore(gclass, wrist):
    if gclass == 'h':
        pose = wrist.pose()
    else:
        pose = vwrist.compose(wrist.inverse()).pose(zthr=0.1, fail=False)
    # TODO: This is pretty random, do better...
    return 3*abs(pose.theta) + abs(pose.y) - abs(pose.x)

def graspClass(wrist):
    # is wrist close enough to horizontal or vertical approach?
    euler = euler_from_matrix(wrist.matrix)
    eps = math.pi/10
    if abs(hu.angleDiff(math.pi/2, euler[1])) < eps:
        return 'v', (euler[0], math.pi/2, euler[2])
    elif abs(euler[0]) < eps and abs(euler[1]) < eps:
        return 'h', (0., 0., euler[2])
    else:
        return None, None

def mapToClass(euler, wrist):
    rot = euler_matrix(*euler)
    rot[:,3] = wrist.matrix[:,3]        # set translation
    rot[2,3] = round(2*rot[2,3], 1)*0.5 # round to 0.05
    return hu.Transform(rot)

def flipLtoR(pose):
    m = pose.matrix.copy()
    m[1,3] = -m[1,3]
    return hu.Transform(m)

# Try to get the angles to produce an actual grasp of the specified
# class (horizontal or vertical).

'''
def refineAnglesOpt(conf, cart, armChainName, euler):
    def confFromVec(confVec):
        return conf.set(armChainName, confVec.tolist())
    def vecFromConf(conf):
        return np.array(conf[armChainName])
    limits = conf.robot.limits([armChainName])
    target = cart.set(armChainName, mapToClass(euler, cart[armChainName]))
    # function to minimize - takes confVec, returns number
    def cost(confVec):
        return confFromVec(confVec).cartConf().distance(target, [armChainName])
    # Do optimization.  Possibly should do multiple restarts?
    cv0 = np.array(conf[armChainName])
    optVec, fopt, _, _, _ = fmin_slsqp(cost, cv0, bounds=limits, disp=0, full_output=True)
    if fopt < 0.01:
        return optVec.tolist()
'''

def refineAngles(conf, cart, armChainName, euler):
    target = cart.set(armChainName, mapToClass(euler, cart[armChainName]))
    nconf = conf.robot.inverseKin(target, conf)
    if None in nconf.values():
        return None
    return nconf[armChainName]
