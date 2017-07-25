import math
import itertools
from random import uniform
from collections import deque
import numpy as np
import PyR2.hu as hu
import PyR2.shapes as shapes
from PyR2.transformations import quaternion_slerp
from scipy.optimize import fmin_bfgs, fmin_slsqp

Ident = hu.Transform(np.eye(4, dtype=np.float64)) # identity transform

# fkCount, fkCache, placeCount, placeCache
confCacheStats = [0, 0, 0, 0]

# Controls size of confCache - bigger cache leads to faster motion
# planning, but makes Python bigger, which can lead to swapping.
maxConfCacheSize = 150*10**3
# print '*** maxConfCacheSize', maxConfCacheSize, '***'

# (additions, deletions)
confCacheUpdateStats = [0, 0]
def printStats():
    print 'maxConfCacheSize', maxConfCacheSize
    print 'confCacheStats = (fkCount, fkCache, placeCount, placeCache)\n', confCacheStats
    print 'confCacheUpdateStats = (additions, deletions)\n', confCacheUpdateStats

# These are not specific to a particular robot
class GenericRobot:
    def cacheReset(self):
        self.confCache = {}
        self.confCacheKeys = deque([])  # in order of arrival

    def limits(self, chainNames = []):
        return itertools.chain(*[self.chains.chainsByName[name].limits()\
                                 for name in chainNames])

    def limitsDict(self, chainNames = []):
        return {name:self.chains.chainsByName[name].limits()\
                for name in chainNames}

    def randomConf(self, moveChains=None, defaultConf=None):
        conf = defaultConf or self.makeJointConf({})
        for chainName in (moveChains or self.moveChainNames):
            if not chainName in self.chains.chainsByName: continue
            conf = conf.set(chainName,
                            self.chains.chainsByName[chainName].randomValues())
        return conf

    def randomStep(self, conf, moveChains=None, stepSize=0.1):
        limits = self.limitsDict(moveChains)
        for chainName in (moveChains or self.moveChainNames):
            if not chainName in self.chains.chainsByName: continue
            vals = conf.get(chainName)
            nvals = vals[:]
            lim = limits[chainName]
            for i in range(len(vals)):
                limi = lim[i]
                step = stepSize*(limi[1] - limi[0])
                offset = uniform(-step, step)
                print (step, offset), 
                nvals[i] = max(lim[i][0], min(lim[i][1], vals[i]+offset))
            conf = conf.set(chainName, nvals)
        return conf

    def addChainDependencies(self, moveChains):
        mc = []
        for chain in moveChains:
            mc.extend(self.chainDependencies[chain])
        return list(set(mc))

    def baseShape(self, c):
        parts = dict([(o.name(), o) for o in c.placement().parts()])
        return parts[self.baseChainName]

    def armAndGripperShape(self, c, hand, attached):
        parts = dict([(o.name(), o) for o in c.placement(attached=attached).parts()])
        armShapes = [parts[self.armChainNames[hand]],
                     parts[self.gripperChainNames[hand]]]
        if attached and attached[hand]:
            armShapes.append(parts[attached[hand].name()])
        return shapes.Shape(armShapes, None)

    def armShape(self, c, hand):
        parts = dict([(o.name(), o) for o in c.placement().parts()])
        armShapes = [parts[self.armChainNames[hand]]]
        return shapes.Shape(armShapes, None)

    def gripperShape(self, c, hand, attached):
        parts = dict([(o.name(), o) for o in c.placement(attached=attached).parts()])
        gripperShapes = [parts[self.gripperChainNames[hand]]]
        if attached and attached[hand]:
            gripperShapes.append(parts[attached[hand].name()])
        return shapes.Shape(gripperShapes, None)

    def placement(self, conf, getShapes=True, attached=None):
        place, attachedParts, trans = self.placementAux(conf, getShapes, attached)
        if attached and getShapes and any(attached.values()):
            return shapes.Shape(place.parts() + [x for x in attachedParts.values() if x],
                                place.origin(),
                                name=place.name()), trans
        else:
            return place, trans

    def updateConfCache(self, key, value):
        while len(self.confCacheKeys) > maxConfCacheSize:
            confCacheUpdateStats[1] += 1
            oldKey = self.confCacheKeys.popleft()
            del(self.confCache[oldKey])
        confCacheUpdateStats[0] += 1
        self.confCacheKeys.append(key)
        self.confCache[key] = value

    def placementAux(self, conf, getShapes=True, attached=None):
        # The placement is relative to the state in some world (provides the base frame)
        # Returns a Shape object and a dictionary of frames for each sub-chain.
        frame = Ident
        shapeChains = getShapes
        key = (conf, frame, True if getShapes==True else tuple(getShapes))
        confCacheStats[0 if not getShapes else 2] += 1
        # confCache = (fkCount, fkCache, placeCount, placeCache)
        if key in self.confCache:
            confCacheStats[1 if not getShapes else 3] += 1
            place, trans = self.confCache[key]
        else:
            place, trans = self.chains.placement(frame, conf, getShapes=shapeChains)
            self.updateConfCache(key, (place, trans))
        attachedParts = {h:None for h in conf.robot.handNames()}
        if attached and any(attached.values()):
            for hand in attached:
                if attached[hand]:
                    attachedParts[hand] = attached[hand].applyTrans(trans[self.wristFrameNames[hand]])
        return place, attachedParts, trans

    def gripperPlace(self, conf, hand, wrist, robotPlace=None):
        confWrist = conf.cartConf()[self.armChainNames[hand]]
        if not robotPlace:
            robotPlace = conf.placement()
        gripperChain = self.gripperChainNames[hand]
        shape = (part for part in robotPlace.parts() if part.name() == gripperChain).next()
        return shape.applyTrans(confWrist.inverse()).applyTrans(wrist)

    def completeJointConf(self, conf, baseConf):
        cfg = conf.copy()
        for cname in self.chainNames:
            if not cname in conf.keys():
                # use current chain confs as default.
                cfg = cfg.set(cname, baseConf[cname])
        return cfg

    def stepAlongLine(self, q_f, q_i, stepSize,
                      forward = True, moveChains = None, useCart = None, c_f = None):
        moveChains = moveChains or self.moveChainNames
        q = q_i.copy()
        # Reverse the order of chains when working on the "from the goal" tree.
        if useCart:
            c_i = q_i.cartConf()
        for chainName in self.chainNames if forward else self.chainNames[::-1]:
            if not chainName in moveChains or \
               q_f[chainName] == q_i[chainName]: continue
            if useCart and chainName in useCart:
                jv = stepTowardsTarget(chainName, c_f, q_f, c_i, q_i, stepSize)
            else:
                jv = self.chains.chainsByName[chainName].stepAlongLine(list(q_f[chainName]),
                                                                       list(q_i[chainName]),
                                                                       stepSize)
            return q.set(chainName, jv) # only move one chain at a time...
        return q_i

    def interpolate(self, q_f, q_i, ratio, stepSize,
                    forward = True, moveChains = None):
        moveChains = moveChains or self.moveChainNames
        q = q_i.copy()
        notSmall = False
        for chainName in self.chainNames if forward else self.chainNames[::-1]:
            if not chainName in moveChains or \
               q_f[chainName] == q_i[chainName]: continue
            jv, s = self.chains.chainsByName[chainName].interpolate(list(q_f[chainName]),
                                                                       list(q_i[chainName]),
                                                                       ratio, stepSize)
            # if any of the steps are not small, then overall is not small.
            if not s: notSmall = True
            q = q.set(chainName, jv)
        return q, not notSmall

    def distConf(self, q1, q2, moveChains=None):
        total = 0.
        for chainName in moveChains or self.chainNames:
            if chainName in q1.conf and chainName in q2.conf:
                total += self.chains.chainsByName[chainName].dist(q1[chainName], q2[chainName])
        return total

    def distConfAbs(self, q1, q2, moveChains=None, dmax=1.0e6):
        total = 10*abs(hu.angleDiff(q1.baseConf()[-1], q2.baseConf()[-1]))
        for chainName in moveChains or self.chainNames:
            if chainName in q1.conf and chainName in q2.conf:
                td = self.chains.chainsByName[chainName].distAbsLess(q1[chainName],
                                                                     q2[chainName],
                                                                     dmax-total)
                if td is None: return None
                total += td
        return total

    # "normalize" the angles...
    def normConf(self, target, source):
        cByN = self.chains.chainsByName
        for chainName in self.chainNames:
            if not chainName in target.conf or not chainName in source.conf: continue
            if target[chainName] == source[chainName]: continue
            target = target.set(chainName, cByN[chainName].normalize(target[chainName],
                                                                     source[chainName]))
        return target

    def cartInterpolators(self, conf_f, conf_i, minLength):
        c_f = conf_f.cartConf()
        c_i = conf_i.cartConf()
        return cartInterpolatorsAux(c_f, c_i, conf_i, minLength)

    def trivialInverseKin(self, cart, defaultConf):
        return defaultConf              # needs to be provided

    def inverseKinGeneric(self, cart, defaultConf, maxRestarts=10):
        return inverseKinConf(cart, defaultConf,
                              maxRestarts=maxRestarts)

    def handViewPoints(self):
        assert None, 'Not implemented'

def interpPose(pose_f, pose_i, minLength, ratio=0.5):
    if isinstance(pose_f, (tuple, list)):
        return [f*ratio + i*(1-ratio) for (f,i) in zip(pose_f, pose_i)], \
               all([abs(f-i)<=minLength for (f,i) in zip(pose_f, pose_i)])
    else:
        pr = pose_f.point()*ratio + pose_i.point()*(1-ratio)
        qr = quaternion_slerp(pose_i.quat().matrix, pose_f.quat().matrix, ratio)
        return hu.Transform(None, pr.matrix, qr), \
               pose_f.near(pose_i, minLength, minLength)

# Since this interpolates all chains, we call robot.inverseKin
def cartInterpolatorsAux(c_f, c_i, conf_i, minLength, depth=0):
    if depth > 10:
        raw_input('cartInterpolators depth > 10')
    robot = conf_i.robot
    if c_f == c_i: 
        conf = robot.inverseKin(c_f, conf_i)
        conf['robotHead'] = conf_i['robotHead']
        return [conf]
    newVals = {}
    terminal = True
    for chain in c_i.conf:
        new, near = interpPose(c_f.conf[chain], c_i.conf[chain], minLength)
        newVals[chain] = new
        terminal = terminal and near
    if terminal: return []        # no chain needs splitting
    cart = c_f.robot.makeCartConf(newVals)
    conf = robot.inverseKin(cart, conf_i)
    headChain = robot.headChainName
    conf.conf[headChain] = conf_i[headChain]
    final = []
    if all(conf.conf.values()):
        final = cartInterpolatorsAux(c_f, cart, conf, minLength, depth+1)
        if final != None:
            init = cartInterpolatorsAux(cart, c_i, conf_i, minLength, depth+1)
            if init != None:
                final.append(conf)
                final.extend(init)
    return final

def stepTowardsTarget(chainName, c_f, q_f, c_i, q_i, stepSize, zthr=0.1):
    # diffPose must be a Pose (x,y,z,t)
    # diffPose = c_f[chainName].compose(c_i[chainName].inverse())
    diffPose = c_i[chainName].inverse().compose(c_f[chainName])
    diffs = diffPose.pose(zthr=zthr).xyztTuple()
    length = sum([d**2 for d in diffs])**0.5
    step = stepSize/length
    # print 'step', step, 'length', length
    if length == 0. or step >= 1.:
        return q_f[chainName]
    # We're going to interpolate diffPose and apply it to c_i, which need not be a pose
    vals = [d*step for d in diffs]
    cc = q_i.robot.makeCartConf({})
    cc = cc.set(chainName, c_i[chainName].compose(hu.Pose(*vals)))
    q = q_i.robot.inverseKin(cc, q_f) #  may have Nones in values
    return q[chainName]

cacheGenIK = True
useTanMapping = False
cache = {}

def inverseKinConf(cart, defaultConf, maxRestarts=10):
    chainNames = [cart.chainName(c) for c in cart.keys() if 'Arm' in c]
    if not chainNames:
      return defaultConf # no arms to solve for
    if cacheGenIK:
        key = frozenset(tuple([(k, cart[k].matrix.data) for k in chainNames]))
        if cache.get(key, None) is not None:
            print 0, '... found invKin in cache'
            val = cache[key]
            if True:                    # all(val.conf.values()):
                conf = defaultConf
                for k in chainNames:
                    conf = conf.set(k, val[k])
                return conf
    else:
        key = False

    # Functions that map back and forth from internal vector representation
    confFromVec, vecFromConf, chainNames, chainIndices, chainLimits = kinMappingFunctions(cart, defaultConf)

    # function to minimize - takes confVec, returns number
    def cost(confVec):
        return confFromVec(confVec).cartConf().distance(cart, chainNames)
    # Do optimization.  Possibly should do multiple restarts?
    guessConf = defaultConf
    for trial in range(maxRestarts):
        # guessConf.prettyPrint('guessConf '+str(trial))
        cv0 = vecFromConf(guessConf)
        if useTanMapping:
            optVec, fopt, _, _, _, _, _ = fmin_bfgs(cost, cv0, full_output=True)
        else:
            bounds = []
            for chain in chainNames:
                bounds.extend(chainLimits[chain])
            eps = 0.001
            bounds = [(lo+eps, hi-eps) for (lo,hi) in bounds]
            optVec, fopt, _, _, _ = fmin_slsqp(cost, cv0, bounds=bounds, disp=0, full_output=True)
        if fopt < 0.01:
            break
        else:
            guessConf = defaultConf.robot.randomConf(chainNames, guessConf)
    if fopt < 0.01:
        print trial, '... found invkin'
        conf = confFromVec(optVec)
    else:
        print trial, '... did not find invKin'
        conf = defaultConf
        for chain in chainNames:
            conf = conf.set(chain, None)
    if key: cache[key] = conf
    return conf

def kinMappingFunctions(cart, defaultConf):
    robot = defaultConf.robot
    chainNames = [cart.chainName(c) for c in cart.keys() if 'Arm' in c]
    chainLimits = robot.limitsDict(chainNames)
    chainIndices = {}
    ind = 0
    for chain in chainNames:
        n = len(defaultConf[chain])
        chainIndices[chain] = range(ind, ind+n)
        ind += n
    # Shared vector...
    vec = np.zeros(ind)
    def confFromVec(confVec):
        conf = defaultConf
        for chain in chainNames:
            lim = chainLimits[chain]
            ind = chainIndices[chain]
            val = []
            for i,j in zip(ind, range(len(lim))):
                if useTanMapping:
                    val.append(mapFromTan(confVec[i], lim[j]))
                else:
                    val.append(confVec[i])
            conf = conf.set(chain, val)
        return conf
    def vecFromConf(conf):
        for chain in chainNames:
            lim = chainLimits[chain]
            ind = chainIndices[chain]
            val = conf[chain]
            for i,j in zip(ind, range(len(lim))):
                if useTanMapping:
                    vec[i] = mapToTan(val[j], lim[j])
                else:
                    vec[i] = val[j]
        return vec

    return confFromVec, vecFromConf, chainNames, chainIndices, chainLimits

def mapToTan(t, limits):
    (tlo, thi) = limits
    mapped = ((2*t - thi - tlo)/(thi-tlo))
    assert -1 <= mapped <= 1
    return math.tan((math.pi/2) * mapped)

def mapFromTan(z, limits):
    (tlo, thi) = limits
    return ((thi - tlo)/math.pi)*math.atan(z) + ((thi + tlo)/2.)

'''
    # estimated gradient - takes confVec, returns gradVec
    def costGrad(confVec):
        c0 = cost(confVec)
        for i in range(N):
            original = confVec[i]
            confVec[i] += delta
            c1 = cost(confVec)
            confVec[i] = original
            gradVec[i] = (d1 - d0) / delta
        return gradVec
'''
