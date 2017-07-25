import pdb
import math
import numpy as np
import time
from autil.globals import glob
import geometry.hu as hu
from autil.mmUtil import removeDuplicateConfs, minDist, confGradientDescent
import graphics.windowManager3D as wm
from random import randrange

hands = (0, 1) # left, right

class ChainRRT:
    def __init__(self, shWorld, initConf, goalConf,
                 allowedViol, moveChains, stepSize,
                 useCart=False, ignoreShadows=False):
        if glob.debugRRT: print 'Setting up RRT'
        if allowedViol is None: allowedViol=hu.Violations()
        self.initConf = initConf
        self.shWorld = shWorld
        robot = self.shWorld.robot
        self.moveChainNames = moveChains or goalConf.keys()
        self.collisionMoveChainNames = robot.addChainDependencies(moveChains)
        self.ignoreShadows = ignoreShadows
        # useCart is a list of arm chains that move and have attached object
        self.useCart = []
        if useCart:
            initCartConf = robot.forwardKin(initConf)
            if goalConf:
                goalCartConf = robot.forwardKin(goalConf)
            for h in ('right', 'left'):
                if self.shWorld.attached[h] \
                       and (robot.armChainNames[h] in self.moveChainNames):
                    chainName = robot.armChainNames[h]
                    self.useCart.append(chainName)
                    # Make sure that the init and goal are valid (related by a pose)
                    if goalConf:
                        if not goalCartConf[chainName].compose(initCartConf[chainName].inverse()).pose(fail=False, zthr=0.1):
                            # goalConf.draw('W', 'magenta')
                            # raw_input('Cartesian planning requested wih invalid confs')
                            self.useCart = []
                            break
        if glob.debugRRT and self.useCart:
            print 'Calling RRT with cartesian interpolation for', self.useCart
        self.Ta = Tree(shWorld, initConf, True, allowedViol, moveChains, stepSize,
                       useCart=self.useCart, ignoreShadows=self.ignoreShadows)
        if goalConf:
            self.Tb = Tree(shWorld, goalConf, False,
                           allowedViol, moveChains, stepSize, useCart=self.useCart,
                           ignoreShadows=self.ignoreShadows)

    # Optimize??
    def randConf(self):
        return self.shWorld.robot.randomConf(self.moveChainNames, self.initConf)

    def normConf(self, target, source):
        return self.shWorld.robot.normConf(target, source)

    def swapTrees(self):
        if self.Ta.size > self.Tb.size:
            self.Ta, self.Tb = self.Tb, self.Ta

    # the qx variables denote confs; the nx variable denote nodes
    def buildBiTree(self, K=1000):
        """Builds the RRT and returns either a pair of nodes (one in each tree)
        with a common configuration or FAILURE."""
        if glob.debugRRT: print 'Building BiRRT'
        n_new = self.Ta.stopNode(self.Tb.root.conf, self.Ta.root)
        if eqChains(n_new.conf, self.Tb.root.conf, self.moveChainNames):
            if glob.debugRRT: print 'Found direct path'
            na_new = self.Ta.addNode(n_new.conf, self.Ta.root)
            nb_new = self.Tb.addNode(n_new.conf, self.Tb.root)
            return (na_new, nb_new)
        for i in xrange(K):
            if glob.debugRRT:
                # if i % 100 == 0: print i
                print i
            q_rand = self.randConf()
            na_near = self.Ta.nearest(q_rand)
            # adjust continuous angle values
            q_rand = self.normConf(q_rand, na_near.conf)
            na_new = self.Ta.stopNode(q_rand, na_near)

            if not na_new is na_near:
                nb_near = self.Tb.nearest(na_new.conf)
                nb_new = self.Tb.stopNode(na_new.conf, nb_near)
                if glob.debugRRTDisplay:
                    wm.getWindow('W').clear(); self.shWorld.draw('W')
                    na_new.conf.draw('W', 'blue')
                    nb_new.conf.draw('W', 'green')
                    raw_input('Go?')
                if eqChains(na_new.conf, nb_new.conf, self.moveChainNames):
                    if glob.debugRRT:
                        print 'BiRRT: Goal reached in' +\
                              ' %s iterations' %str(i)
                    return (na_new, nb_new)
            self.swapTrees()
        if glob.debugRRT:
            print '\nBiRRT: Goal not reached in' + ' %s iterations\n' %str(K)
        return 'FAILURE'

    # the qx variables denote confs; the nx variable denote nodes
    def buildTree(self, goalTest, K=1000):
        """Builds the RRT and returns either a node or FAILURE."""
        if glob.debugRRT: print 'Building RRT'
        if goalTest(self.Ta.root.conf):
            return self.Ta.root
        for i in xrange(K):
            if glob.debugRRT:
                if i % 100 == 0: print i
            q_rand = self.randConf()
            na_near = self.Ta.nearest(q_rand)
            # adjust continuous angle values
            q_rand = self.normConf(q_rand, na_near.conf)
            na_new = self.Ta.stopNode(q_rand, na_near, maxSteps = glob.maxStopNodeSteps)
            if goalTest(na_new.conf):
                return na_new
        if glob.debugRRT:
            print '\nRRT: Goal not reached in' + ' %s iterations\n' %str(K)
        return 'FAILURE'

    def safePath(self, qf, qi, display = False):
        q = self.Ta.stopNode(qf, Node(qi, None, None),
                             addNodes=False, display=display).conf
        return eqChains(q, qf, self.moveChainNames)

    def tracePath(self, node):
        path = [node]; cur = node
        while cur.parent != None:
            cur = cur.parent
            path.append(cur)
        return path

    def findGoalPath(self, goalTest, K=None):
        node = self.buildTree(goalTest, K)
        if node is 'FAILURE': return 'FAILURE'
        path = self.tracePath(node)[::-1]
        goalValues = [goalTest(c.conf) for c in path]
        if True in goalValues:
            goalIndex = goalValues.index(True)
            return path[:goalIndex+1]       # include up to first True
        else:
            print 'Empty solution in findGoalPath'
            raw_input('Continue anyway?')
            return 'FAILURE'
    
    def findPath(self, K=None):
        sharedNodes = self.buildBiTree(K)
        if sharedNodes is 'FAILURE': return 'FAILURE'
        pathA = self.tracePath(sharedNodes[0])
        pathB = self.tracePath(sharedNodes[1])
        if pathA[0].tree.init:
            return pathA[::-1] + pathB
        elif pathB[0].tree.init:
            return pathB[::-1] + pathA
        else:
            raise Exception, "Neither path is marked init"

def eqChains(conf1, conf2, moveChains):
    return all(conf1[c]==conf2[c] for c in moveChains)

idnum = 0
class Node:
    def __init__(self, conf, parent, tree, inter=False):
        global idnum
        self.inter = inter
        self.id = idnum; idnum += 1
        self.conf = conf
        self.children = []
        self.parent = parent
        self.tree = tree
    def __str__(self):
        return 'Node:'+str(self.id)
    def __hash__(self):
        return self.id

class Tree:
    def __init__(self, shWorld, conf, init,
                 allowedViol, moveChains, stepSize, useCart=False, ignoreShadows=False):
        self.useCart = useCart
        self.shWorld = shWorld
        self.root = Node(conf, None, self)
        self.nodes = [Node(conf, None, self)]
        self.size = 0
        self.init = init
        self.allowedViol = allowedViol
        self.moveChainNames = moveChains
        self.collisionMoveChainNames = shWorld.robot.addChainDependencies(moveChains)
        self.stepSize = stepSize
        self.ignoreShadows = ignoreShadows

    def addNode(self, conf, parent, inter=False):
        n_new = Node(conf, parent, self, inter=inter)
        parent.children.append(n_new)
        self.nodes.append(n_new)
        self.size += 1
        return n_new

    def nearest(self, q):               # q is conf
        robot = self.shWorld.robot
        near_d = float('inf')
        near_node = None
        for node in self.nodes:
            d = robot.distConfAbs(q, node.conf, self.collisionMoveChainNames, near_d)
            if not d is None and d < near_d:
                near_d = d
                near_node = node
        return near_node
    
    def stopNode(self, q_f, n_i,
                 addNodes = True,
                 maxSteps = 1000,
                 display = False):
        q_i = n_i.conf
        if eqChains(q_f, q_i, self.moveChainNames): return n_i
        step = 0
        if self.useCart:
            c_i = q_i.cartConf()
            c_f = q_f.cartConf()
            for chain in self.useCart:
                # diff = c_f[chain].compose(c_i[chain].inverse())
                diff = c_i[chain].inverse().compose(c_f[chain])
                # Change the target to be related by a pose
                c_f = c_f.set(chain, c_i[chain].compose(nearestPose(diff)))
        else:
            c_f = None

        while True:
            if maxSteps:
                if step >= maxSteps:
                    return n_i
            step += 1
            q_new = self.shWorld.robot.stepAlongLine(q_f, q_i, self.stepSize,
                                                     forward = self.init,
                                                     moveChains = self.moveChainNames,
                                                     useCart = self.useCart, c_f = c_f)
            if all(x is not None for x in q_new.conf.values()) and \
                   safeConfChain(q_new, self.shWorld, self.allowedViol,
                                 (None if step==1 else self.collisionMoveChainNames),
                                 self.ignoreShadows):
                # We may choose to add intermediate nodes to the tree or not.
                if addNodes:
                    n_new = self.addNode(q_new, n_i, inter=True);
                else:
                    n_new = Node(q_new, n_i, self, inter=True)
                if eqChains(q_f, q_new, self.moveChainNames):
                    n_new.inter = False
                    return n_new
                n_i = n_new
                q_i = n_i.conf
            else:                       # a collision
                n_i.inter = False
                return n_i

    def __str__(self):
        return 'TREE:['+str(len(self.size))+']'

def nearestPose(trans):
    mat = trans.matrix
    nz = np.array([0., 0., 1.])
    x = mat[:3,0]
    ny = np.cross(nz, x)
    nx = np.cross(ny, nz)
    theta = math.atan2(nx[1], nx[0])
    return hu.Pose(mat[0,3], mat[1,3], mat[2,3], theta)

def safeConfChain(conf, shWorld, allowedViol,
                  collisionMoveChains=None, ignoreShadows=False):
    viol = shWorld.confViolations(conf, moveChains=collisionMoveChains)
    # if we're already in violation with one object, then ignore that collision.

    # Forgive all shadow collisions!!  So, just check the obstacles.
    ans = viol and \
          (set(viol.allObstacles()) <= set(allowedViol.allObstacles()) if ignoreShadows \
           else set(viol.allObstacles() + viol.allShadows()) \
                 <= set(allowedViol.allObstacles() + allowedViol.allShadows()))
          
    if glob.debugSafeConf:
        if True:
            wm.getWindow('W').clear(); shWorld.draw('W')
            conf.draw('W', 'blue')
            print 'viol', viol
            print 'allowedViol', allowedViol
            raw_input('safeConf ='+str(ans))
    return ans

def runRRT(shWorld, initConf, destConf, allowedViol, moveChains, maxIter, failIter, ignoreShadows):
    nodes = 'FAILURE'
    failCount = -1
    while nodes == 'FAILURE' and failCount < (failIter or glob.failRRTIter):
        rrti = ChainRRT(shWorld, initConf, destConf,
                        allowedViol, moveChains, glob.rrtInterpolateStepSize,
                        useCart=glob.useCart, ignoreShadows=ignoreShadows)
        nodes = rrti.findPath(K = maxIter or glob.maxRRTIter)
        failCount += 1
    if glob.debugRRTFailed:
        if failCount > 0:
            print '    RRT has failed', failCount, 'times'
            pdb.set_trace()
    if nodes == 'FAILURE':
        return None, None
    else:
        return nodes, allowedViol

def runRRTSplit(shWorld, initConf, destConf, allowedViol, moveChains,
                maxIter, failIter, ignoreShadows):
    robot = initConf.robot
    armNames = robot.armChainNames.keys()
    if robot.armChainNames and \
           all(robot.armChainNames[h] in moveChains for h in armNames):
        chain1 = chain2 = None
        if len(armNames) == 2:
            cr = robot.armChainNames['right']
            stowr = robot.armStowAngles['right']
            cl = robot.armChainNames['left']
            stowl = robot.armStowAngles['left']
            if destConf[cr] == stowr and initConf[cr] != stowr:
                chain1 = cr; chain2 = cl
            elif  destConf[cl] == stowl and initConf[cl] != stowl:
                chain2 = cr; chain1 = cl
        if chain1 and chain2:

            if glob.debugRRT and any(shWorld.attached[h] for  h in armNames):
                print '*** RRT Split with held object in hand!', shWorld.attached

            other = [c for c in moveChains if c not in robot.chainDependencies[chain1]]
            assert chain2 in other

            print '** RRT Split', moveChains, chain1, chain2, other

            t1 = initConf.set(chain1, destConf[chain1])
            n1, v1 = runRRT(shWorld, initConf, t1, allowedViol, [chain1],
                            maxIter, failIter, ignoreShadows)
            if n1:
                n2, v2 = runRRT(shWorld, n1[-1].conf, destConf, allowedViol, other,
                                maxIter, failIter, ignoreShadows)
                if n2:
                    return n1 + n2, v1.update(v2)
                else:
                    return None, None
    return runRRT(shWorld, initConf, destConf, allowedViol, moveChains,
                  maxIter, failIter, ignoreShadows)

def planRobotPathChain(shWorld, initConf, destConf, allowedViol, moveChains,
                       maxIter = None, failIter = None, safeCheck = True,
                       backOff = False, ignoreShadows = False):
    q1 = initConf; q2 = destConf
    if backOff and glob.distanceBackoffRRT:
        chains = [chain for chain in q1.conf \
                  if chain in q2.conf \
                  and max([abs(x-y) > 1.0e-6 for (x,y) in zip(q1.conf[chain], q2.conf[chain])])]
        path1, _ = backOffPath(shWorld, q1, chains) # from q1 to backoff
        path2, _ = backOffPath(shWorld, q2, chains) # from q2 to backoff
    else:
        path1 = [q1]
        path2 = [q2]

    path, viol = planRobotPathChainAux(shWorld, path1[-1], path2[-1], allowedViol, moveChains,
                                       maxIter = maxIter, failIter = failIter, safeCheck = safeCheck,
                                       ignoreShadows = ignoreShadows)
    if viol:
        return interpolatePath(path1 + path + path2[::-1]), viol
    else:
        return [], viol

def planRobotPathChainAux(shWorld, initConf, destConf, allowedViol, moveChains,
                          maxIter = None, failIter = None, safeCheck = True,
                          ignoreShadows = False):
    if glob.debugRRT:
        print 'rrt moveChains', moveChains
    startTime = time.clock()
    if allowedViol is None:
        v1 = shWorld.confViolations(destConf)
        v2 = shWorld.confViolations(initConf)
        if v1 and v2:
            allowedViol = v1.update(v2)
        else:
            return [], None
    if safeCheck:
        for name, conf in (('initial', initConf), ('final', destConf)):
            if not safeConfChain(conf, shWorld, allowedViol):
                if glob.debugRRT or glob.debugRRTFailed:
                    print 'RRT: not safe enough at', name, 'conf... continuing'
                return [], None
    if initConf == destConf:
        return [initConf, destConf], allowedViol
    nodes, newViol = runRRTSplit(shWorld, initConf, destConf, allowedViol,
                                 moveChains, maxIter, failIter, ignoreShadows)
    
    rrtTime = time.clock() - startTime

    if nodes is None:
        print 'Failed path in %.3f'%rrtTime, 'secs'
        return [], None
    if glob.debugRRT:
        print 'Found path in %.3f'%rrtTime, 'secs'
    path = [c.conf for c in nodes]
    finalViol = allowedViol.update(newViol)

    if glob.debugVerifyRRTPath:
        verifyPath(shWorld, interpolatePath(path), allowedViol,
                   'interp rrt:'+str(moveChains))
    return path, allowedViol

def planRobotGoalPathChain(shWorld, initConf, goalTest, allowedViol, moveChains,
                           maxIter = None, failIter = None,
                           ignoreShadows = False):
    startTime = time.clock()
    if allowedViol==None:
        v = shWorld.confViolations(initConf)
        if v:
            allowedViol = v
        else:
            return [], None
    if not safeConfChain(initConf, shWorld, allowedViol):
        if glob.debugRRT:
            print 'RRT: not safe enough at initial position...',
            print 'viol =', shWorld.confViolations(initConf)
            pdb.set_trace()
        else:
            return [], None
    nodes = 'FAILURE'
    failCount = -1                      # not really a failure the first time
    while nodes == 'FAILURE' and failCount < (failIter if failIter is not None else glob.failRRTIter):
        rrt = ChainRRT(shWorld, initConf, None, allowedViol, moveChains,
                       glob.rrtInterpolateStepSize, ignoreShadows=ignoreShadows,
                       useCart=glob.useCart)
        nodes = rrt.findGoalPath(goalTest, K = maxIter or glob.maxRRTIter)
        failCount += 1
        if glob.debugRRT or failCount > 0 and failCount % 10 == 0:
            print 'RRT Failed', failCount, 'times in planRobotGoalPath'
    rrtTime = time.clock() - startTime

    if failCount == (failIter or glob.failRRTIter):
        print 'Failed goal path in %.3f'%rrtTime, 'secs'
        if glob.debugFailGoalPath: pdb.set_trace()
        return [], None
    if glob.debugRRTTime: print 'RRT (Goal) time %.3f'%rrtTime
    path = [c.conf for c in nodes]
    # Verify that only the moving chain is moved.
    for chain in initConf.conf:
        if chain not in moveChains:
            assert all(initConf.conf[chain] == c.conf[chain] for c in path)
    if glob.debugVerifyRRTPath:
        verifyPath(shWorld, interpolatePath(path), allowedViol, 'interp rrt:'+chain)
    return interpolatePath(path), allowedViol

## Utilities

def interpolate(q_f, q_i, stepSize=None, moveChains=None, maxSteps=300):
    if stepSize is None:
        stepSize = glob.rrtInterpolateStepSize
    return list(interpolateGen(q_f, q_i, stepSize=glob.rrtInterpolateStepSize,
                               moveChains=moveChains, maxSteps=maxSteps))

def interpolateGen(q_f, q_i, stepSize=None, moveChains=None, maxSteps=300):
    if stepSize is None:
        stepSize = glob.rrtInterpolateStepSize
    robot = q_f.robot
    path = [q_i]
    q = q_i
    step = 0
    moveChains = moveChains or q_f.keys()
    yield q_i
    while q != q_f:
        if step > maxSteps:
            raw_input('interpolate exceeded maxSteps')
        qn = robot.stepAlongLine(q_f, q, stepSize, moveChains=moveChains)
        if eqChains(q, qn, moveChains): break
        q = qn
        path.append(q)
        yield q
        step += 1
    if eqChains(path[-1], q_f, moveChains):
        path.pop()
    path.append(q_f)
    if len(path) > 1 and not(path[0] == q_i and path[-1] == q_f):
        raw_input('Path inconsistency')
    yield q_f

def interpolatePath(path, stepSize=None):
    if stepSize is None:
        stepSize = glob.rrtInterpolateStepSize
    interpolated = []
    for i in range(1, len(path)):
        qf = path[i]
        qi = path[i-1]
        confs = interpolate(qf, qi, stepSize=stepSize)
        if glob.debugRRT: print i, 'path segment has', len(confs), 'confs'
        interpolated.extend(confs)
    return removeDuplicateConfs(interpolated)


def verifyPath(shWorld, prob, path, allowedViol, msg):
    obst = shWorld.getObjectShapes()
    attached = shWorld.attached
    drawn = False
    allowed = allowedViol.allObstacles() + allowedViol.allShadows()
    win = wm.getWindow('W')
    for conf in path:
        if glob.debugVerifyRRTPath:
            win.clear(); shWorld.draw('W')
            drawn = True
            # conf.draw('W')
            # win.update()
            # sleep(0.01)
        if any(o not in allowed and robotShape.collides(o) for o in obst):
            if not drawn:
                win.clear(); shWorld.draw('W')
                drawn = True
            colliders = [o for o in obst if robotShape.collides(o)]
            robotColliders = [p for p in robotShape.parts() \
                              if [o for o in colliders if o.collides(p)]]
            robotShape.draw('W')
            for o in robotColliders: o.draw('W', 'red')
            for o in colliders: o.draw('W', 'red')
            print msg, 'coll:', [o.name() for o in robotColliders], [o.name() for o in colliders],
            print 'safeConf=', safeConfChain(conf, shWorld, allowedViol)
            pdb.set_trace()
    return True

def smoothPath(shWorld, path, verbose=False, moveChains=None, nsteps = 100, npasses = 20):
    if len(path) < 3:
        return path
    win = wm.getWindow('W')
    if verbose: print 'Path has %s points'%str(len(path)), '... smoothing'
    input = removeDuplicateConfs(path)
    if len(input) < 3:
        return path
    checked = set([])
    outer = 0
    count = 0
    step = 0
    if verbose: print 'Smoothing...'
    while outer < npasses:
        if verbose:
            print '  Start smoothing pass', outer, 'len=', len(input)
        smoothed = []
        for p in input:
            if not smoothed or smoothed[-1] != p:
                smoothed.append(p)
        n = len(smoothed)
        while count < nsteps and n > 2:
            if verbose: print 'step', step, ':', 
            if n < 1:
                print 'smooth', 'Path is empty!'
                return removeDuplicateConfs(path)
            i = randrange(n)
            j = randrange(n)
            if j < i: i, j = j, i 
            step += 1
            if verbose: print i, j, len(checked)
            if j-i < 2 or \
                (smoothed[j], smoothed[i]) in checked:
                count += 1
                continue
            else:
                checked.add((smoothed[j], smoothed[i]))
            if verbose:
                win.clear(); shWorld.draw(prob, 'W')
                for k in range(i, j+1):
                    smoothed[k].draw('W', 'blue')
                print 'smooth', 'Testing'
            if safeInterpolation(smoothed[j], smoothed[i], shWorld, moveChains, verbose):
                count = 0
                if verbose:
                    print 'smooth', 'Safe'
                    win.clear(); shWorld.draw('W')
                    for k in range(i+1)+range(j,len(smoothed)):
                        smoothed[k].draw('W', 'blue')
                    print 'smooth', 'remaining'
                smoothed[i+1:j] = []
                n = len(smoothed)
                if verbose: print 'Smoothed path length is', n
            else:
                count += 1
        outer += 1
        if outer < npasses:
            count = 0
            if verbose: print 'Re-expanding path'
            input = removeDuplicateConfs(interpolatePath(removeDuplicateConfs(smoothed)))
    if verbose:
        print 'Final smooth path len =', len(smoothed), 'dist='

    ans = interpolatePath(removeDuplicateConfs(smoothed))
    assert ans[0].nearEqual(path[0]) and ans[-1].nearEqual(path[-1]),'Inconsistent path'
    return ans

minStep = glob.rrtInterpolateStepSize
def safeInterpolation(qf, qi, shWorld, moveChains=None, verbose=False):
    for conf in interpolate(qf, qi, stepSize=minStep):
        newViol = shWorld.confViolations(conf, moveChains=moveChains)
        if newViol is None or newViol.weight() > 0.:
            if verbose: conf.draw('W', 'red')
            return False
        else:
            if verbose: conf.draw('W', 'green')
    return True

def removeDuplicateConfs(path):
    inp = []
    for p in path:
        if not inp or not inp[-1].nearEqual(p):
            inp.append(p)
    return inp

###
# Backoff - a trajectory to increase the min distance from robot to obstacles
# Don't even whisper the words "potential field"...
##

def retractPath(shWorld, path):
    attached = shWorld.attached
    conf = path[0]
    placement = conf.placement(attached=attached)
    objects = [obj for obj in shWorld.objectShapes.values() \
               if minDist(conf, None, [obj], placement)[0] < backOffTarget]
    if objects:
        bestVal = 0.
        for i, cf in enumerate(path):
            val, _ = minDist(cf, attached, objects)
            if val >= backOffTarget or val < bestVal:
                break
            bestVal = val
    else:
        i = 0
    print 'retractPath', 'path length', len(path), 'retract start to', i
    conf = path[-1]
    placement = conf.placement(attached=attached)
    objects = [obj for obj in shWorld.objectShapes.values() \
               if 1.0e-6 < minDist(conf, None, [obj], placement)[0] < backOffTarget]
    if objects:
        bestVal = 0
        for k, cf in enumerate(path[::-1]):
            val, _ = minDist(cf, attached, objects)
            if val >= backOffTarget or val < bestVal:
                break
            bestVal = val
        assert k > 0
        j = len(path) - (k+1)
    else:
        j = len(path)
    print 'retractPath', 'path length', len(path), 'retract end to', j
    return path[:i], path[i:j], path[j:]

backOffCache = {}
backOffTarget = 0.25
def backOffPath(shWorld, conf, moveChains,
                maxSteps=20, backOff=backOffTarget):
    def shadowp(x): return '_shadow' in x.name()
    robot = conf.robot
    attached = shWorld.attached
    placement = conf.placement(attached=attached)
    # Look only at shadows, since they're bigger
    candidates = [o for o in shWorld.objectShapes.values() if shadowp(o)]
    objects = [obj for obj in candidates \
               if minDist(conf, None, [obj], placement)[0] < backOff]
    if glob.debugBackOff:
        print 'objects', [obj.name() for obj in objects]
    if not objects:
        ans = [conf], backOff
        return ans
    def distFun(c, chains):
        d, o = minDist(c, attached, objects)
        return -d                       # we want to increase this...
    def validFun(c):
        return c is conf or shWorld.confViolations(c) is not None
    if robot.baseChainName in moveChains:
        chainSeq = [[x for x in moveChains if (x != robot.baseChainName and \
                                               x in robot.armChainNames.values())],
                    [robot.baseChainName]]
    else:
        chainSeq = [[x for x in moveChains if x in robot.armChainNames.values()]]
    path, val = confGradientDescent(conf, chainSeq, distFun, validFun,
                                    maxSteps=maxSteps, target=-backOff)
    if glob.debugBackOff:
        wm.getWindow('W').clear(); shWorld.draw('W');
        conf.draw('W', attached=attached, color='blue')
        path[-1].draw('W', attached=attached, color='red')
        raw_input('backOff')

    ans = path, val
    return ans
