import pdb
import math, random, copy
import numpy as np
import geometry.hu as hu
from graphics.colors import RGBToPyColor, HSVtoRGB
from autil.miscUtil import Hashable
from autil.globals import glob
from geometry.gjk2 import gjkDist
from geometry.shapes import Box

# Rect grasp set.
# !! Could also have a cylindrical one.  What's the API?
class GDesc(Hashable):
    def __init__(self, obj, frame, dx, dy, dz):
        self.obj = obj                    # ??
        self.frame = frame
        # y is approach direction, x is transverse, z is grip
        self.dx = dx                      # half-widths of a box
        self.dy = dy
        self.dz = dz
        Hashable.__init__(self)
    def desc(self):
        return (self.obj, self.frame, (self.dx, self.dy, self.dz))

def combineViols(viols):
    v = hu.Violations()
    for viol in viols:
        if viol == None:
            return None
        v = v.update(viol)
    return v

def violationsFromNames(ws, names):
    shapes = ws.objectShapes
    (obstacles, shadows, heldObstacles, heldShadows) = names
    return hu.Violations(obstacles = [shapes[o] for o in obstacles],
                         shadows = [shapes[o] for o in shadows],
                         heldObstacles = [[shapes[o] for o in ho] for ho in heldObstacles],
                         heldShadows = [[shapes[o] for o in hs] for hs in heldShadows])

def movingChains(q1, q2):
    return [chain for chain in q1.conf \
            if chain in q2.conf \
            and any([abs(x-y) > 1.0e-6 \
                     for (x,y) in zip(q1.conf[chain], q2.conf[chain])])]

class NextColor:
    def __init__(self, num, s = .4, v = .99):
        self.max = num
        self.current = 0
        self.s = s
        self.v = v
    def next(self):
        h = (float(self.current) / self.max) * 360
        self.current = (self.current + 1) % self.max
        col = RGBToPyColor(HSVtoRGB(h, self.s, self.v))
        return col

def restFaceIndex(shape):
    origin = shape.origin()
    for f, ff in enumerate(shape.faceFrames()):
        ffo = origin.compose(ff)
        if abs(1.0 - ffo.matrix[2,2]) < 0.01:
            return f

def getSupportPose(shape, supportFace=None):
    if supportFace is None:
        supportFace = restFaceIndex(shape)
    pose = shape.origin().compose(shape.faceFrames()[supportFace])
    # print 'origin frame\n', shape.origin().matrix
    # print 'supportPose\n', pose.matrix
    return pose

colorGen = NextColor(20)

def drawPath(path, viol=None, attached=None):
    c = colorGen.next()
    for conf in path:
        conf.draw('W', color=c, attached=attached)
    if viol:
        for v in viol.obstacles:
            v.draw('W', 'red')
        for v in viol.shadows:
            v.draw('W', 'orange')

######################################################################
        
memoizerBufferN = 5
class Memoizer:
    def __init__(self, name, generator, values = None):
        self.name = name
        self.generator = generator
        self.values = values if values else [name]
        self.i = 0
    def __iter__(self):
        return self
    def copy(self):
        # shares the generator and values list, only index differs.
        if glob.debugGenMemoize:
            print '  * Re-initializing gen =', self.name, 'with', len(self.values), 'values'
        new = Memoizer(self.name, self.generator, self.values)
        return new
    def next(self):
        if self.i < len(self.values)-1:
            i = self.i
            self.i += 1
            if glob.debugGenMemoize:
                print '  * for gen =', self.name, 'returning', self.values[i+1]
            return self.values[i+1]
        else:
            val = self.generator.next()
            self.values.append(val)
            self.i += 1
            if glob.debugGenMemoize:
                print '  * for gen =', self.name, 'returning', val
            return val

# A decorator function for restartable generator
# http://stackoverflow.com/questions/1376438/how-to-make-a-repeating-generator-in-python
# Suggestion from Gustavo...

def multigen(gen_func):
    class _multigen(object):
        def __init__(self, *args, **kwargs):
            self.__args = args
            self.__kwargs = kwargs
        def __iter__(self):
            return gen_func(*self.__args, **self.__kwargs)
    return _multigen

def bigAngleWarn(conf1, conf2, thr = math.pi/8.):
    if not glob.debugBigAngleChange: return
    for chain in conf.robot.armChainNames.values():
        joint = 0
        for angle1, angle2 in zip(conf1[chain], conf2[chain]):
            if abs(hu.angleDiff(angle1, angle2)) >= thr:
                print 'bigAngleChange', chain, joint, angle1, angle2
            joint += 1

tiny = 1.0e-6
def inside(shape, reg, strict=True, buffer=0.):
    bbox = shape.bbox()
    if strict:
        if buffer == 0.:
            buffer = -0.001             # a little inside
    else:
        buffer = min([0.5*(bbox[1][i] - bbox[0][i]) for i in [0,1]])
    return any(all(insideAux(s, r, buffer) \
                   for s in shape.parts()) for r in reg.parts())

def insideAux(shape, reg, buffer=tiny):
    # all([np.all(np.dot(reg.planes(), p) <= 1.0e-6) for p in shape.vertices().T])
    verts = shape.getVertices()
    for i in xrange(verts.shape[1]):
        if not np.all(np.dot(reg.planes(), verts[:,i].reshape(4,1)) <= buffer):
            return False
    return True

def bboxGridCoords(bb, n=5, z=None, res=None):
    eps = 0.001
    ((x0, y0, z0), (x1, y1, z1)) = tuple(bb)
    x0 += eps; y0 += eps
    x1 -= eps; y1 -= eps
    if res:
        dx = res
        dy = res
        nx = int(float(x1 - x0)/res)
        ny = int(float(y1 - y0)/res)
        if nx*ny > n*n:
            for point in bboxGridCoords(bb, n=n, z=z, res=None):
                yield point
    else:
        dx = float(x1 - x0)/n
        dy = float(y1 - y0)/n
        nx = ny = n
    if z is None: z = z0
    for i in range(nx+1):
        x = x0 + i*dx
        for j in range(ny+1):
            y = y0 + j*dy
            yield np.array([x, y, z, 1.])

# Assume an implicit grid 0.01 on a side
def bboxRandomGridCoords(bb, n=5, z=None, step=0.01):
    ((x0, y0, z0), (x1, y1, z1)) = tuple(bb)
    x0 = round(x0, 2)
    y0 = round(y0, 2)
    x1 = round(x1, 2)
    y1 = round(y1, 2)
    if z is None: z = z0
    nx = int(round((x1-x0)/step))
    ny = int(round((y1-y0)/step))
    maxn = nx*ny
    vals = set([])
    count = 0
    while count < n and len(vals) < maxn:
        i = random.randint(0, nx)
        j = random.randint(0, ny)
        if (i, j) in vals: continue
        else:
            vals.add((i,j))
            count += 1
            x = round(x0 + i*step, 2)
            y = round(y0 + j*step, 2)
            yield np.array([x, y, z, 1.])

def bboxRandomCoords(bb, n=20, z=None):
    ((x0, y0, z0), (x1, y1, z1)) = tuple(bb)
    if z is None: z = z0
    for i in xrange(n):
        x = random.uniform(x0, x1)
        y = random.uniform(y0, y1)
        yield np.array([x, y, z, 1.])

# prob is probability of generatng a grid point, 1-prob is for random
def bboxMixedCoords(bb, prob, n=20, z=None):
    grid = bboxRandomGridCoords(bb, n=n, z=z)
    rand = bboxRandomCoords(bb, n=n, z=z)
    for i in xrange(n):
        if random.random() <= prob:
            yield next(grid, next(rand))
        else:
            yield next(rand)

def otherHand(hand):
    return 'left' if hand == 'right' else 'right'

def checkCache(cache, key, valueFn):
    if key not in cache:
        cache[key] = valueFn(*key)
    return cache[key]

def checkAndUpdateCache(cache, key, value):
    if key in cache:
        return True, cache[key]
    else:
        cache[key] = val
        return False, None

def baseConfWithin(bc1, bc2, delta):
    (x1, y1, t1) = bc1
    (x2, y2, t2) = bc2
    if len(delta) == 4:
        (dx, dy, dz, dt) = delta
    else:                               # (dist, angle)
        dx = dy = delta[0]
        dt = delta[1]
    return abs(x2-x1) <= dx and \
           abs(y2-y1) <= dy and \
           abs(hu.angleDiff(t2,t1)) <= dt

def handConfWithin(c1, c2, hand, delta):
    c1CartConf = c1.cartConf()
    c2CartConf = c2.cartConf()
    robot = c1.robot
    handChainName = robot.armChainNames[hand]
    return within(c1CartConf[handChainName],c2CartConf[handChainName],delta)
    
def within(a, b, delta = (.01, .01, .01, .01)):
    if isinstance(a, list):
        dd = delta[0]                # !! hack
        return all([abs(a[i] - b[i]) <= dd \
                    for i in range(min(len(a), len(b)))])
    else:
        return a.withinDelta(b, delta)

def confWithinCart(c1, c2, delta):

    def withinDelta(x, y):
        return within(x, y, delta)

    if not all([d >= 0 for d in delta]):
        print 'confWithin', 'negative delta', d
        return False

    # We only care whether | conf - targetConf | <= delta
    # Check that the moving frames are all within specified delta
    c1CartConf = c1.cartConf()
    c2CartConf = c2.cartConf()
    robot = c1.robot

    cartConfWithin = [withinDelta(c1CartConf[x],c2CartConf[x]) \
                                           for x in robot.moveChainNames]
    if not all(cartConfWithin):
        print 'confWithin', 'cart conf failed'
        return False
    
    # Also be sure the head angles are the same
    if not all(hu.nearAngle(h1, h2, delta[-1]) for (h1, h2) \
               in zip(c1[c1.robot.headChainName], c2[c2.robot.headChainName])):
        return False

    # This ignores grippers

    return True

def confWithin(c1, c2, delta, ignoreChains=[]):

    def withinDelta(x, y):
        return within(x, y, delta)

    jointAngleNear = 0.01               # will this work on the robot??
    def confClose(v1, v2):
        nearVal = [abs(angle1 - angle2) < jointAngleNear for angle1, angle2 in zip(v1, v2)]
        nearAng = [abs(hu.angleDiff(angle1, angle2)) < jointAngleNear for angle1, angle2 in zip(v1, v2)]
        print v1
        print v2
        print 'nearVal', nearVal
        print 'nearAng', nearAng
        return all(nearVal) or all(nearAng)

    if not all([d >= 0 for d in delta]):
        print 'confWithin', 'negative delta', d
        return False

    robot = c1.robot
    chains = [c for c in c1.keys() if c != robot.baseChainName]

    return baseConfWithin(c1.baseConf(), c2.baseConf(), delta) and \
           all(confClose(c1.get(chain), c2.get(chain)) for chain in chains \
               if chain not in ignoreChains)

def objectGraspFrameAux(graspDescs, grasp, graspMode,
                        faceFrames, restFace, poseMode,
                        robot, hand, origin=False):
    # if we're using the origin, we don't need the restFace or faceFrames
    assert origin or (faceFrames is not None and restFace is not None)
    # Find the robot wrist frame corresponding to the grasp at the placement
    if origin:                          # poseFrame is origin
        objFrame = poseMode
    else:                               # poseFrame is on restFace
        objFrame = poseMode.compose(faceFrames[restFace].inverse())
    # objFrame is the origin of the object
    graspDesc = graspDescs[grasp]
    faceFrame = graspDesc.frame.compose(graspMode)
    centerFrame = faceFrame.compose(hu.Pose(0,0,graspDesc.dz,0))
    graspFrame = objFrame.compose(centerFrame)
    # !! Rotates wrist frame to grasp face frame
    gT = robot.gripperFaceFrame[hand]
    wristFrame = graspFrame.compose(gT.inverse())

    if glob.debugObjectGraspFrame:
        print 'objGrasp', (grasp, graspMode)
        print 'objPlace', (restFace, poseMode)
        print 'objFrame\n', objFrame.matrix
        print 'grasp faceFrame\n', faceFrame.matrix
        print 'centerFrame\n', centerFrame.matrix
        print 'graspFrame\n', graspFrame.matrix
        print 'object wristFrame\n', wristFrame.matrix

    return wristFrame

def robotGraspFrame(pbs, conf, hand):
    robot = pbs.getRobot()
    wristFrame = conf.cartConf()[robot.armChainNames[hand]]
    if glob.debugRobotGraspFrame:
        print 'robot wristFrame\n', wristFrame.matrix
    return wristFrame

def objInHand(shWorld, conf, hand, imagine=True):
    attached = shWorld.attached
    if not attached[hand] and imagine:
        attached = attached.copy()
        tool = conf.robot.toolOffsetX[hand]
        attached[hand] = Box(0.1,0.05,0.1, None, name='virtualObject').applyLoc(tool)
    _, attachedParts = conf.placementAux(attached, getShapes=[])
    return attachedParts[hand]

def removeDuplicateConfs(path):
    inp = []
    for p in path:
        if not inp or not inp[-1].nearEqual(p):
            inp.append(p)
    return inp

def moveChains(conf1, conf2, eps = 1.0e-6):
    return [chain for chain in conf1.conf \
            if chain in conf2.conf \
            and max([abs(hu.angleDiff(x, y)) > eps \
                         for (x,y) in zip(conf1.conf[chain], conf2.conf[chain])])]  

def segmentPathByChains(path):
    path = removeDuplicateConfs(path)
    # find moving chains
    mc = [None]
    for i in range(1, len(path)):
        mc.append(moveChains(path[i-1], path[i], 0.005))
    # group by moving chains
    curChains = None
    curSegment = []
    segments = []
    for conf, chains in zip(path, mc):
        if chains == curChains:
            curSegment.append(conf)
        else:
            segments.append((curChains, curSegment))
            curChains = chains
            curSegment = [conf]
    if curSegment:
        segments.append((curChains, curSegment))
    assert sum([len(x[1]) for x in segments]) == len(path), \
           'Inconsistent segmentation'
    segments = [(chains,seg) for (chains,seg) in segments \
                 if not chains is []]
    return segments

def minDist(conf, attached, objects, placement=None, moveChains=None):
    if placement is None:
        placement = conf.placement(attached=attached,
                                   getShapes=(moveChains if moveChains else True))
    # Look at the whole placement - it may be better to focus on
    # moving chains only?
    placePrims = placement.toPrims()
    minD = float('inf')
    minO = None
    for obj in objects:
        for o in obj.toPrims():
            for p in placePrims:
                d = gjkDist(o, p)**0.5
                if d < minD:
                    minD = d
                    minO = obj.name()
    return minD, minO

def diagnoseCollision(a, b):
    for ap in a.toPrims():
        for bp in b.toPrims():
            if ap.collides(bp):
                ap.draw('W', 'red'); bp.draw('W', 'red')
                print 'Collision', ap.name(), bp.name()

def replaceZ(thing, val):
    newThing = [thing[i] if i != 2 else val for i in range(4)]
    return tuple(newThing) if isinstance(thing, tuple) else newThing

def confGrad(conf, moveChains, delta, fun):
    grad = {}
    for chain in moveChains:
        qV = conf[chain][:]
        grad[chain] = len(conf[chain])*[0.]
        for i, q in enumerate(conf[chain]):
            try:
                qV[i] = q - delta
                newConf1 = conf.set(chain, qV[:])
                assert conf.robot.chains.valid(newConf1)
                d1 = fun(newConf1, [chain])
                qV[i] = q + delta
                newConf2 = conf.set(chain, qV[:])
                assert conf.robot.chains.valid(newConf2)
                d2 = fun(newConf2, [chain])
                qV[i] = q
                grad[chain][i] = (d2 - d1)/(2*delta)
            except:
                qV[i] = q
                grad[chain][i] = 0.0
    return grad

def confGradStep(conf, grad, step):
    newConf = conf.copy()
    for chain in grad:
        s = 0.02                        # make sure we don't step beyond
        for q in grad[chain]:
            if q != 0.:
                s = min(s, abs(step)/abs(q))
        newConf.conf[chain] = [q-(s*g) for q,g in zip(newConf[chain], grad[chain])]
    return newConf

def confGradientDescent(conf, chainSeq, fun, vfun,
                        maxSteps=20, target=-float('inf'), step=0.01):
    cf = conf
    path = [cf]
    for chains in chainSeq:
        val = fun(cf, chains)
        for it in xrange(maxSteps):
            grad = confGrad(cf, chains, 0.01, fun)
            new_cf = confGradStep(cf, grad, 0.01)
            if not (conf.robot.chains.valid(new_cf) and vfun(new_cf)):
                break
            try:
                new_val =  fun(new_cf, chains)
            except:
                new_val = val
            if glob.debugConfGD:
                print 'chains', chains, 'val', val, 'new_val', new_val
            if new_val is None or \
                   new_val <= target or \
                   new_val > (val+0.001):
                break
            val = new_val
            cf = new_cf
            path.append(new_cf)
    return path, val


# This is like getVertices for shape but with subdivision
def getSubdivisionVertices(shape, res = 0.1):
    # return an array of verts by constructing subdivision surfaces.
    # primSubdivide returns (verts, faces), all we want are the verts.
    return np.hstack([primSubdivide(prim, res)[0] for prim in shape.toPrims()])

def primSubdivide(prim, res):
    def vdist(v1, v2):
        return math.sqrt(sum([(v1[i]-v2[i])**2 for i in range(3)]))
    def maxEdgeDist(face):
        return max([vdist(verts[face[i]], verts[face[(i+1)%len(face)]]) \
                    for i in range(len(face))])
    faces = prim.faces()[:]
    vertsArray = prim.getVertices()
    verts = [vertsArray[:,i] for i in xrange(vertsArray.shape[1])]
    split = True
    done = set([])
    # print 'prim=', prim
    while split:
        # print '    faces=', len(faces), 'verts=', len(verts)
        split = False
        edge_mid = {}                      # edge midpoints for one pass ((vi, vj) : vk)
        for faceIndex in xrange(len(faces)):
            if faceIndex in done: continue
            done.add(faceIndex)
            face = faces[faceIndex]     # vert indices for face
            if maxEdgeDist(face) <= res:
                continue
            # print faceIndex, 'face=', face, 'max dist', maxEdgeDist(face)
            fl = len(face)
            lv = []                     # list of midpoint indices for this face
            # Create verts at the midpoints of each edge, if not there already
            for fi in range(fl):
                fi1 = (fi + 1)%fl       # edge indices (fl, fl1)
                vi = face[fi]; vi1 = face[fi1]
                if (vi, vi1) in edge_mid:
                    lv.append(edge_mid[(vi, vi1)])
                elif (vi1, vi) in edge_mid:
                    lv.append(edge_mid[(vi1, vi)])
                else:
                    # New midpoint
                    verts.append(0.5*verts[vi] + 0.5*verts[vi1]) # the midpoint
                    lv.append(len(verts)-1)                      # add to midpoints for face
                    edge_mid[(vi, vi1)] = lv[-1]                 # record in edges
            for fi in range(fl):
                # Triangular face centered on each original vertex
                faces.append((lv[fi-1], face[fi], lv[fi]))
            faces.append(lv)            # the central face
            split = True

    return np.vstack(verts).T, faces

print 'Loaded amber/mmUtil.py'
