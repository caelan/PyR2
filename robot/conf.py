import math
from autil.utils import prettyString
import geometry.hu as hu
import numpy as np

# This behaves like a dictionary, except that it doesn't support side effects.
class RobotJointConf:
    def __init__(self, conf, robot):
        global confIdnum
        self.conf = conf
        self.robot = robot
        self.baseName = self.robot.baseChainName
        self.items = None
        self.strStored = {True:None, False:None}
    def copy(self):
        return RobotJointConf(self.conf.copy(), self.robot)
    def values(self):
        return self.conf.values()
    def keys(self):
        return self.conf.keys()
    def get(self, name, default = None):
        if name in self.conf:
            return self.conf[name]
        else:
            return default
    def set(self, name, value):
        assert value is None or isinstance(value, list), \
               'Illegal value for JointConf set, should be a list'
        c = self.copy()
        c.conf[name] = value
        return c
    def minimalConf(self, hand):
        armChainName = self.robot.armChainNames.get(hand, None)
        if armChainName:
            return (tuple(self.baseConf()), tuple(self.conf[armChainName]))
        else:
            return (tuple(self.baseConf()),)
    # Abstract interface...
    def basePose(self):
        # type: () -> object
        base = self.conf[self.baseName]
        return hu.Pose(base[0], base[1], 0.0, base[2])
    def setBasePose(self, basePose):
        xyzt = list(basePose.pose().xyztTuple())
        return self.setBaseConf((xyzt[0], xyzt[1], xyzt[3]))
    def setBaseConf(self, baseConf):
        assert isinstance(baseConf, (tuple, list)) and len(baseConf) == 3, \
               'Illegal baseConf'
        return self.set(self.baseName, list(baseConf))
    def baseConf(self):                 # (x, y, theta)
        return tuple(self.conf[self.baseName])
    def headConf(self):
        return self.conf[self.robot.headChainName]
    def setGrip(self, hand, width):
        return self.set(self.robot.gripperChainNames[hand],
                        [min(width, self.robot.gripMax)])
    def cartConf(self):
        return self.robot.forwardKin(self)
    def placement(self, attached=None, getShapes=True):
        return self.robot.placement(self, attached=attached, getShapes=getShapes)[0]
    def placementAux(self, attached=None, getShapes=True):
        place, attachedParts, trans = self.robot.placementAux(self, attached=attached,
                                                              getShapes=getShapes)
        return place, attachedParts
    def distance(self, other, chains = None):
        if chains is None: chains = self.keys()
        return sum(sum([abs(hu.angleDiff(x,y)) for (x,y) in zip(self.conf[c],other.conf[c])]) \
                   for c in chains if c in other.conf)
    def draw(self, win, color='black', attached=None):
        pl = self.placement(attached=attached)
        if pl: pl.draw(win, color)
    def prettyString(self, eq = True):
        if not eq:
            # If we don't need to maintain equality, just print the base
            if self.strStored[eq] is None:
                self.strStored[eq] = 'JointConf('+prettyString(self.conf[self.baseName], eq)+')'
        else:
            if self.strStored[eq] is None:
                self.strStored[eq] = 'JointConf('+prettyString(self.conf, eq)+')'
        return self.strStored[eq]
    def prettyPrint(self, msg='Conf:'):
        print msg
        for key in sorted(self.conf.keys()):
            print '   ', key, prettyString(self.conf[key])
    def confItems(self):
        if not self.items:
            self.items = frozenset([(chain, tuple(self.conf[chain])) for chain in self.conf])
            # self.items = prettyString(self.conf)
        return self.items

    def nearEqual(self, other):
      if not hasattr(other, 'conf'): return False
      if not set(self.conf.keys()) == set(other.conf.keys()): return False
      return not any([any([abs(x-y) > 1.0e-6 for (x,y) in zip(self.conf[chain],
                                other.conf[chain])]) for chain in self.conf])

    def __str__(self):
        return 'JointConf('+str(self.conf)+')'
    def ss(self):
        return 'J%s'%(prettyString(self.conf[self.baseName]))
    def __getitem__(self, name):
        return self.conf[name]
    def __hash__(self):
        return hash(self.confItems())
    def __eq__(self, other):
        if not hasattr(other, 'conf'): return False
        return self.confItems() == other.confItems()
    def __ne__(self, other):
        if not hasattr(other, 'conf'): return True
        return not self.confItems() == other.confItems()

class RobotCartConf(RobotJointConf):
    def __init__(self, conf, robot):
        self.conf = conf
        self.robot = robot
        self.baseName = self.robot.baseChainName
        self.items = None

    def chainName(self, name):
        if 'Frame' in name:
            return name[:-5]
        else:
            return name
    def frameName(self, name):
        if 'Frame' in name or 'Gripper' in name or 'Torso' in name:
            return name
        else:
            return name + 'Frame'
    def get(self, name, default = None):
        name = self.frameName(name)
        if name in self.conf:
            return self.conf[self.frameName(name)]
        else:
            return default
    def set(self, name, value):
        c = self.copy()
        name = self.frameName(name)
        if 'Gripper' in name or 'Torso' in name:
            if isinstance(value, list):
                c.conf[name] = value
            else:
                c.conf[name] = [value]
        else:
            assert value is None or isinstance(value, hu.Transform), \
                   'Illegal value for CartConf set'
            c.conf[name] = value
        return c
    def rem(self, name):
        name = self.frameName(name)
        if not name in self.conf:
            return self
        c = self.copy()
        del(c.conf[name])
        return c
    def basePose(self):
        # type: () -> object
        return self.conf[self.baseName].pose()
    def confItems(self):
        if not self.items:
            vals = []
            for chain in self.conf:
                val = self.conf[chain]
                if isinstance(val, list):
                    vals.append(tuple(val))
                elif isinstance(val, hu.Transform):
                    vals.append(repr(val))
                else:
                    vals.append(val)
            self.items = frozenset(vals)
        return self.items
    def distance(self, other, chains = None):
        d = 0.
        if chains is None: chains = self.keys()
        for c in chains:
            if not other.get(c): continue
            if isinstance(self.get(c), hu.Transform):
                strans = self.get(c)
                otrans = other.get(c)
                d += np.linalg.norm(otrans.matrix[:3,3] - strans.matrix[:3,3])
                dot = np.dot(otrans.quat().matrix, strans.quat().matrix)
                dot = max(-1.0, min(1.0, dot))
                d += 2*abs(math.acos(abs(dot)))
            else:
                d += sum(abs(hu.angleDiff(x,y)) for (x,y) in zip(self.get(c),other.get(c)))
        return d
    def prettyPrint(self, msg='Cart Conf:'):
        print msg
        for key in sorted(self.conf.keys()):
            if isinstance(self.conf[key], hu.Transform):
                print '   ', key, '\n', self.conf[key].matrix
            else:
                print '   ', key, prettyString(self.conf[key])
    def copy(self):
        return RobotCartConf(self.conf.copy(), self.robot)
    def __getitem__(self, name):
        return self.conf[self.frameName(name)]
