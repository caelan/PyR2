cimport geometry.hu as hu
import geometry.hu as hu
import geometry.transformations as transf
import copy
import math
import random 
import numpy as np
cimport numpy as np
from cpython cimport bool
import xml.etree.ElementTree as ET
from collections import OrderedDict

from autil.globals import glob
from geometry.hu cimport Transform, angleDiff, fixAnglePlusMinusPi
from geometry.shapes cimport Shape
from geometry.geom import bboxUnion, bboxOrigin, vertsBBox
from geometry.gjk2 import confViolationsOS, OS_Array, printOSa
import graphics.windowManager3D as wm
from geometry.chains cimport linkVerts, compileChainFramesOS, MultiChain, chainCmp, Chain, Movable, \
  Planar, XYZT, Permanent, RevoluteDoor, GripperChain, Joint, Prismatic, Revolute, \
  normalizedAngleLimits, General, Rigid, ChainFrameOS

emptyAttachedCCd = {'left':None, 'right':None}
cdef tuple handName = ('left', 'right')
cdef Transform Ident = Transform(np.eye(4))

############################################################
# Objects
############################################################
class Universe:
    def __init__(self):
        self.objectDataBase = {}
        self.objectTypes = {}

    def setObjectType(self, prefix, otype=None):
        if otype is None: otype = prefix
        self.objectTypes[prefix] = otype
    def getObjectType(self, obj):
        if obj in self.objectTypes.values():
            return obj
        for prefix in self.objectTypes:
            if obj[0:len(prefix)] == prefix:
                return self.objectTypes[prefix]
        return None
    def setObjectTypeData(self, obj, dataType, value):
        if dataType not in self.objectDataBase:
            self.objectDataBase[dataType] = {}
        objType = self.getObjectType(obj)
        self.objectDataBase[dataType][objType] = value
    def getObjectTypeData(self, obj, dataType, default=None):
        if dataType in self.objectDataBase:
            dataBase = self.objectDataBase[dataType]
            objType = self.getObjectType(obj)
            if objType not in dataBase:
                print objType, 'not in', dataBase
            return dataBase.get(objType, default)
        else:
            return None

class World:
    def __init__(self, universe):
        self.world = self                 # so x.world works..
        self.objects = {}
        self.objectShapes = {}
        self.regions = {}
        self.robot = None
        self.workspace = None
        self.workspaceRegion = None
        self.workspaceWalls = None
        # pointClouds (4xn numpy arrays) for each type.
        self.typePointClouds = {}
        # Basically a cache for constructor calls
        self.objectArchetypeShapes = {}      # type names, shape
        self.objectArchetypeRegions = {}      # type names, regions list
        self.universe = universe

    def getObjectType(self, obj):
        if obj in self.universe.objectTypes.values():
            return obj
        for prefix in self.universe.objectTypes:
            if obj[0:len(prefix)] == prefix:
                return self.universe.objectTypes[prefix]
        return None

    def setObjectType(self, prefix, otype):
        self.universe.objectTypes[prefix] = otype

    def getObjectTypeData(self, obj, dataType, default=None):
        if dataType == 'faceFrames':
            return self.getFaceFrames(obj)
        if dataType in self.universe.objectDataBase:
            dataBase = self.universe.objectDataBase[dataType]
            objType = self.getObjectType(obj)
            return dataBase.get(objType, default)
        else:
            return None

    def setObjectTypeData(self, obj, dataType, value):
        if dataType not in self.universe.objectDataBase:
            self.universe.objectDataBase[dataType] = {}
        objType = self.getObjectType(obj)
        self.universe.objectDataBase[dataType][objType] = value

    def copy(self):
        cw = copy.copy(self)
        cw.world = cw
        cw.objects = self.objects.copy()
        cw.objectShapes = self.objectShapes.copy()
        cw.regions = self.regions.copy()
        return cw
        
    def addObject(self, obj):             # obj is Chain or MultiChain
        if isinstance(obj, Chain):
            obj = MultiChain(obj.name, [obj])
        else:
            raw_input('addObject expects a Chain')
        self.objects[obj.name] = obj

    def addObjectShape(self, objShape):
        self.addObject(Movable(objShape.name(), 'root', objShape))

    def delObject(self, name):
        print 'Deleting object', name, 'from the world!'
        if name in self.objects: del self.objects[name]
        if name in self.regions: del self.regions[name]
        if name in self.objectShapes: del self.objectShapes[name]

    def addObjectRegion(self, objName, regName, regShape, regTr):
        if objName in self.regions:
            matches = [(r, s, t) for (r,s,t) in self.regions[objName] if r==regName]
            if matches and not (matches[0] == (regName, regShape, regTr)):
                print 'trying to add:', (regName, regShape, regTr)
                print '     existing:', matches[0]
                raw_input('Inconsitent region definitions')
                return
            self.regions[objName].append((regName, regShape, regTr))
        else:
            self.regions[objName] = [(regName, regShape, regTr)]

    def addObjectShapeAndRegions(self, objName):
        otype = self.getObjectType(objName)
        (shape, spaces) = self.getObjectTypeData(otype, 'constructor')(name=objName)
        self.addObjectShape(shape)
        for (reg, pose) in spaces:
            self.addObjectRegion(objName, reg.name(), reg, pose)

    def addObjectRegions(self, objName):
        otype = self.getObjectType(objName)
        (_, spaces) = self.getObjectTypeData(otype, 'constructor')(name=objName)
        for (reg, pose) in spaces:
            self.addObjectRegion(objName, reg.name(), reg, pose)

    def getRegionNames(self):
        result = []
        for thing in self.regions.values():
            for (name, _, _) in thing:
                result.append(name)
        return frozenset(result)

    def setRobot(self, robot):
        self.robot = robot

    def getObjectShapeAtOrigin(self, objName):
        def simplify(shape, depth=0):
            if depth >= 10:
                print 'Simplify loop:', objName, shape, shape.parts()
                raw_input('Huh')
                return shape
            if shape.name() == objName and \
               len(shape.parts()) == 1 and  \
               shape.parts()[0].name() == objName:
                return simplify(shape.parts()[0], depth=depth+1)
            else:
                return shape
        if objName in self.objectShapes:
            return self.objectShapes[objName]

        obj = self.objects[objName]
        chains = obj.chainsInOrder
        assert isinstance(chains[0], Movable), 'getObjectShapeAtOrigin, not Movable'
        assert all([isinstance(chain, Permanent) for chain in chains[1:]]), 'getObjectShapeAtOrigin, not Permanent'
        conf = dict([[objName, [Ident]]] +\
                    [[chain.name, []] for chain in chains[1:]])
        shape = obj.placement(Ident, conf)[0]
        
        # Make sure that we remove unnesessary nestings
        finalShape = simplify(shape)
        self.objectShapes[objName] = finalShape
        return finalShape

    def getFaceFrames(self, obj):
        if obj in self.objectShapes:
            return self.getObjectShapeAtOrigin(obj).faceFrames()
        
    def __str__(self):
        return 'World(%s)'%({'objects':self.objects})
    __repr__ = __str__

class WorldState:
    def __init__(self, world, robot=None):
        self.robot = robot or world.robot
        self.world = world.copy() # This is a state of this world
        # Below is "state"
        self.objectConfs = {}     # object name -> object conf
        self.objectProps = {}     # object name -> property dictionary
        self.robotConf = None
        # Below is some derived information from this state
        self.frames = {'root': Ident}  # frame name -> Transform
        self.objectShapes = {}              # keep track of placements
        self.regionShapes = {}
        self.robotPlace = None
        self.fixedObjects = set([])           # immovable object names
        self.fixedConf = False                           # if conf fixed?
        # Grasp related
        self.fixedHeld = {'left':False, 'right':False}  # is held obj fixed?
        self.fixedGrasp = {'left':False, 'right':False}  # is grasp fixed?
        self.held = {'left':None, 'right':None}   # object names
        self.grasp = {'left':None, 'right':None}  # transforms
        self.attached = {'left':None, 'right':None} # object shapes
        # for confViolations
        self.objectCC = []
        self.attachedCCd = {'left':None, 'right':None}
        self.objOSa = None
        self.objNamesOSa = None
        self.attOSal = None

    def initializeOS(self):
        if self.robot.OSa is None:
            self.robot.OSa = compileOSa([self.robot.compiledChainsOS])
            if any(self.robot.selfCollideChainNames):
                self.robot.selfCollidePairs = [moveIndices(self.robot.compiledChainsOS,
                                                           names, self.robot.OSa.number) \
                                               for names in self.robot.selfCollideChainNames]
            else:
                self.robot.selfCollidePairs = None
        if self.objOSa is None:
            self.objOSa, self.objNamesOSa = compileOSa(self.objectCC, returnObjNames=True)
        if self.attOSal is None:
            self.attOSal = [compileOSa([self.attachedCCd[hand]]) if self.attachedCCd[hand] else None \
                            for hand in ('left', 'right')]

    def getObjectShapeAtOrigin(self, objName):
        return self.world.getObjectShapeAtOrigin(objName)
    def getObjectShapes(self):
        return self.objectShapes.values()
    def getShadowShapes(self):
        return [shape for shape in self.getObjectShapes() \
                if shape.name()[-7:] == '_shadow']
    def getNonShadowShapes(self):
        return [shape for shape in self.getObjectShapes() \
                if shape.name()[-7:] != '_shadow']

    def selectShapes(self, shadow=None, fixed=None, exceptions=tuple()):
        if shadow is None:                # both
            objShapes = self.getObjectShapes()
        elif shadow is True:
            objShapes = self.getShadowShapes()
        else:
            objShapes = self.getNonShadowShapes()
        exNames = [x if isinstance(x,str) else x.name() \
                   for x in exceptions]
        selected = []
        for s in objShapes:
            sname = s.name()
            if sname in exNames: continue
            if fixed is None or \
                   fixed is True and sname in self.fixedObjects or \
                   fixed is False and (not sname in self.fixedObjects):
                selected.append(s)
        return selected
    
    def copy(self):
        cws = WorldState(self.world)
        cws.objectConfs = self.objectConfs.copy()
        cws.objectProps = self.objectProps.copy()
        cws.frames = self.frames.copy()
        cws.objectShapes = self.objectShapes.copy()
        cws.regionShapes = self.regionShapes.copy()
        cws.robotConf = self.robotConf
        cws.robotPlace = self.robotPlace
        cws.held = self.held.copy()
        cws.grasp = self.grasp.copy()
        cws.attached = self.attached.copy()
        return cws

    def addObjectAndShadow(self, obj, shadow, pose, fixedObject, fixedShadow):
        # Add objects to world and set their pose in state
        self.world.addObjectShape(obj); self.setObjectPose(obj.name(), pose, updateCC=False)
        self.world.addObjectShape(shadow); self.setObjectPose(shadow.name(), pose, updateCC=False)
        if fixedObject: self.fixedObjects.add(obj.name())
        if fixedShadow: self.fixedObjects.add(shadow.name())
        # Do a joint update to CC
        self.objectCC.append(compileObjectFrames(self.objectShapes[obj.name()],
                                                 self.objectShapes[shadow.name()],
                                                 fixedObject, fixedShadow))

    def addFixedObject(self, objName):
        self.fixedObjects.add(objName)

    # attach the object (at its current pose) to gripper (at current conf)
    def attach(self, object, hand='left'):
        if isinstance(object, str):
            objectShape = self.objectShapes[object]
        else:
            objectShape = object
        conf = self.robotConf
        cartConf = conf.cartConf()
        frame = cartConf[conf.robot.armChainNames[hand]]
        obj = objectShape.applyTrans(frame.inverse())
        # self.attachHeld(hand, obj.name(), obj)
        self.attached[hand] = obj
        self.held[hand] = obj.name()

    # detach and return the object (at its current pose) from gripper (at current conf)
    def detach(self, hand='left'):
        obj = self.attached[hand]
        if obj:
            conf = self.robotConf
            cartConf = conf.cartConf()
            frame = cartConf[conf.robot.armChainNames[hand]]
            self.detachHeld(hand)
            return obj.applyTrans(frame)
        else:
            raw_input('Attempt to detach, but no object is attached')

    # Just detach the object, don't return it.
    def detachRel(self, hand='left'):
        if self.attached[hand]:
            self.detachHeld(hand)
        else:
            assert None, 'Attempt to detach, but no object is attached'

    def attachedObj(self, hand='left'):
        obj = self.attached[hand]
        if obj:
            conf = self.robotConf
            cartConf = conf.cartConf()
            frame = cartConf[conf.robot.armChainNames[hand]]
            return obj.applyTrans(frame)
        else:
            return None

    def attachHeld(self, hand, heldObj, heldShape, fixedHeld=None, fixedGrasp=None):
        self.attached[hand] = heldShape
        self.held[hand] = heldObj
        if fixedHeld is not None: self.fixedHeld[hand] = fixedHeld
        if fixedGrasp is not None: self.fixedGrasp[hand] = fixedGrasp
        wristFrameName = self.robot.wristFrameNames[hand]
        wristFrame = Ident.matrix       # straight outta wrist
        chainName = self.robot.armChainNames[hand]
        self.attachedCCd[hand] = (compileAttachedFrames(heldShape, hand,
                                                       wristFrameName, wristFrame,
                                                       fixedHeld, fixedGrasp),
                                  ['attached'],
                                  OrderedDict([(chainName, ['attached'])]),
                                  {'attached': chainName})
        if glob.useGJK2:
            self.attOSal = [compileOSa([self.attachedCCd[hand]]) if self.attachedCCd[hand] else None \
                            for hand in ('left', 'right')]

    def detachHeld(self, hand, fixedHeld=None, fixedGrasp=None):
        self.attached[hand] = None
        self.held[hand] = None
        if fixedHeld is not None: self.fixedHeld[hand] = fixedHeld
        if fixedGrasp is not None: self.fixedGrasp[hand] = fixedGrasp
        self.attachedCCd[hand] = None
        if glob.useGJK2: self.attOSal = None

    def setObjectConf(self, objName, conf):
        obj = self.world.objects[objName]
        if not isinstance(conf, dict):
            conf = {objName:conf}
        # Update the state of the world
        self.objectConfs[objName] = conf
        for chain in obj.chainsInOrder:
            chainPlace, chainTrans = chain.placement(self.frames[chain.baseFname],
                                                     conf[chain.name])
            for part in chainPlace.parts():
                self.objectShapes[part.name()] = part

            if objName in self.world.regions:
                for (regName, regShape, regTr) in self.world.regions[objName]:
                    tr = self.objectShapes[objName].origin().compose(regTr)
                    self.regionShapes[regName] = regShape.applyTrans(tr)
                    if glob.debugAddRegion:
                        print 'obj origin\n', self.objectShapes[objName].origin().matrix
                        print 'regTr\n', regTr.matrix
                        print 'tr\n', tr.matrix
                        print 'obj bb\n', self.objectShapes[objName].bbox()
                        print 'reg bb\n', self.regionShapes[regName].bbox()
                        self.regionShapes[regName].draw('W')
                        print 'adding %s'%regName

            for fname, tr in zip(chain.fnames, chainTrans):
                self.frames[fname] = tr

    # Should there be a getObjectConf?

    def setObjectPose(self, objName, objPose, updateCC = True):
        self.setObjectConf(objName, [objPose])
        if not updateCC: return
        self.objectCC = [x for x in self.objectCC if x[1][0] != objName]
        self.objectCC.append(compileObjectFrames(self.objectShapes[objName],
                                                 None, False, False))

    def getObjectPose(self, objName):
        assert objName in self.objectConfs, 'Unknown object in world'
        confs = self.objectConfs[objName][objName]
        return confs[0] if confs else Ident

    def delObjectState(self, name):
        del self.objectConfs[name]
        del self.objectShapes[name]
        del self.frames[name]

    def setRobotConf(self, conf, fixed=False):
        assert not self.fixedConf, 'Setting a fixed conf'
        robot = self.robot
        # Update the state of the world
        self.robotConf = conf
        self.fixedConf = fixed
        # Include the frames in the world
        placement, frames = robot.placement(conf, attached = self.attached)
        self.robotPlace = placement
        for (fname, tr) in frames.items():
            self.frames[fname] = tr

    def getFrame(self, fname):
        return self.frames[fname]

    def getChainConf(self, cname):
        return self.chainConfs[cname]

    def gripperAtOrigin(self, hand):
        if not self.robotPlace: return None
        robot = self.robot
        gripperChain = robot.gripperChainNames[hand]
        wristFrame = robot.wristFrameNames[hand]
        gripper=next((x for x in self.robotPlace.parts() if x.name()==gripperChain), None)
        # The origin here is the wrist.  
        origin = self.frames[wristFrame].compose(self.robot.gripperTip).pose()
        return gripper.applyTrans(origin.inverse())

    def gripper(self, hand):
        if not self.robotPlace: return None
        robot = self.robot
        gripperChain = robot.gripperChainNames[hand]
        gripper = next((x for x in self.robotPlace.parts() if x.name()==gripperChain), None)
        return gripper

    def base(self):
        if not self.robotPlace: return None
        robot = self.robot
        baseChain = robot.baseChainName
        base = next((x for x in self.robotPlace.parts() if x.name()==baseChain), None)
        return base

    def confViolations(self, conf, **keys):
        def debugDisplay():
            print conf.prettyPrint()
            print 'ans1=', ans1, 'ans2=', ans2, 'ans2set=', set(ans2List)
            print 'moveChains=', keys.get('moveChains', None), 'ignoreAttached', keys.get('ignoreAttached', None)
            self.draw('W')
            conf.draw('W')

        if glob.useGJK2:
            ans2 = self.confViolations2(conf,
                                        moveChains = keys.get('moveChains', None),
                                        ignoreAttached = keys.get('ignoreAttached', None),
                                        prdebug = keys.get('prdebug', False))
        ans1 = ans2
        ans2List = [ans2]
        # if glob.checkGJK2 or not glob.useGJK2:
        #     ans1 = confViolationsChain(conf, self.robot.compiledChainsOS,
        #                                self.attachedCCd, self.objectCC,
        #                                self.robot.selfCollideChainNames,
        #                                **keys)
        # if glob.checkGJK2:
        #     ans2List = [self.confViolations2(conf,
        #                                      moveChains = keys.get('moveChains', None),
        #                                      ignoreAttached = keys.get('ignoreAttached', None),
        #                                      prdebug = keys.get('prdebug', False)) \
        #                 for i in range(10)]
        #     ans2 = ans2List[0]
        #     if len(set(ans2List)) > 1 or ans1 != ans2:
        #         debugDisplay()
        #         self.confViolations2(conf,
        #                              moveChains = keys.get('moveChains', None),
        #                              ignoreAttached = keys.get('ignoreAttached', None),
        #                              prdebug = True)
        #         raw_input('confViolations')

        return ans2 if glob.useGJK2 else ans1

    def checkRobotCollision(self, conf, obj, **keys):
        def debugDisplay():
            conf.prettyPrint()
            print 'ans1=', ans1, 'ans2=', ans2, 'ans2set=', set(ans2List)
            print 'moveChains=', keys.get('moveChains', None), 'ignoreAttached', keys.get('ignoreAttached', None)
            if keys.get('ignoreAttached', False):
                attached = None
            else:
                attached = self.attached
            conf.draw('W', attached=attached)
            obj.draw('W', 'black')

        if glob.useGJK2:
            frames = compileObjectFrames(obj, None, False, False)
            objOSa = compileOSa([frames])
            ans2 = self.checkRobotCollision2(conf, obj, objOSa = objOSa,
                                             moveChains = keys.get('moveChains', None),
                                             ignoreAttached = keys.get('ignoreAttached', None),
                                             prdebug = keys.get('prdebug', False))
        ans1 = ans2
        ans2List = [ans2]
        # if glob.checkGJK2 or not glob.useGJK2:
        #     objCC = [compileObjectFrames(obj, None, False, False)]
        #     viol = confViolationsChain(conf, self.robot.compiledChainsOS,
        #                                self.attachedCCd, objCC,
        #                                self.robot.selfCollideChainNames,
        #                                **keys)
        #     ans1 = (viol is None or viol.weight() > 0)

        # if glob.checkGJK2:
        #     frames = compileObjectFrames(obj, None, False, False)
        #     objOSa = compileOSa([frames])
        #     ans2List = [self.checkRobotCollision2(conf, obj, objOSa = objOSa,
        #                                           moveChains = keys.get('moveChains', None),
        #                                           ignoreAttached = keys.get('ignoreAttached', None),
        #                                           prdebug = keys.get('prdebug', False)) \
        #                 for i in range(10)]
        #     ans2 = ans2List[0]
        #     if len(set(ans2List)) > 1 or ans1 != ans2:
        #         debugDisplay()
        #         # frames = compileObjectFrames(obj, None, False, False, prdebug=True)
        #         # objOSa = compileOSa([frames], prdebug=True)
        #         self.checkRobotCollision2(conf, obj, objOSa = objOSa,
        #                                   moveChains = keys.get('moveChains', None),
        #                                   ignoreAttached = keys.get('ignoreAttached', None),
        #                                   prdebug = True)
        #         raw_input('checkRobotCollision')

        return ans2 if glob.useGJK2 else ans1

    def confViolations2(self, conf, moveChains=None, checkSelfCollide=True,
                        ignoreAttached=False, prdebug=False):
        cdef dict frames
        if self.robot.OSa is None or self.objOSa is None:
            self.initializeOS()
        cdef np.ndarray robMove = moveIndices(self.robot.compiledChainsOS,
                                              moveChains, self.robot.OSa.number)
        # Keep attached objects for moving chains
        if moveChains is None:
            attached = self.attachedCCd
            attOSal = self.attOSal
        elif conf.robot.gripperChainNames:
            attached = {h:(a if conf.robot.gripperChainNames.get(h,None) in moveChains else None)\
                        for (h,a) in self.attachedCCd.iteritems()}
            attOSal = [(a if conf.robot.gripperChainNames.get(h,None) in moveChains else None) \
                       for (h, a) in zip(handName, self.attOSal)]
        else:
            attached = {h:None for h in self.robot.handNames()}
            attOSal = [None, None]
        viol, selfColl = confViolationsOS(conf,
                                          (self.robot.compiledChainsOS,
                                           emptyAttachedCCd if ignoreAttached else attached),
                                          self.robot.OSa,
                                          None if ignoreAttached else attOSal,
                                          self.objOSa,
                                          self.robot.selfCollidePairs if checkSelfCollide else [],
                                          robMove,
                                          self.objectShapes, self.objNamesOSa,
                                          prdebug=prdebug)
        return viol

    def checkConfViolations(self, conf, viol, selfColl, obstacles,
                            ignoreAttached=False, moveChains=None):
        if selfColl: return
        place = conf.placement(attached=(None if ignoreAttached else self.attached),
                               getShapes=moveChains if moveChains else True)
        c = []
        cNames = []
        permanent = False
        for objName in obstacles:
            obj = obstacles[objName]
            if place.collides(obj):
                c.append(obj)
                cNames.append(obj.name())
                if obj.name() in self.fixedObjects:
                    permanent = True
        vc = []
        if viol:
            [vc.append(x.name()) for x in viol.obstacles]
            [vc.append(x.name()) for x in viol.shadows]
            [[vc.append(x.name()) for x in ho] for ho in viol.heldObstacles]
            [[vc.append(x.name()) for x in hs] for hs in viol.heldShadows]
        stop = False
        if viol is None:
            stop = not permanent
        elif set([x.name() for x in c]) != set(vc):
            stop = True
        if stop:
            print 'c', sorted(cNames), 'permanent=', permanent
            print 'vc', sorted(vc) if not viol is None else None
            print 'viol', viol
            print 'fixedHeld', self.fixedHeld, 'fixedObjects', self.fixedObjects
            wm.getWindow('W').clear()
            self.draw('W')
            conf.draw('W', 'blue')
            place.draw('W')
            for o in obstacles.values(): o.draw('W')
            for o in c: obj.draw('W', 'red')
            raw_input('Mismatch')
            return True

    def checkRobotCollision2(self, conf, obj,
                             objOSa=None, objCC=None,
                             moveChains=None, checkSelfCollide=True, ignoreAttached=False,
                             prdebug=False):
        cdef dict frames
        if self.robot.OSa is None or self.objOSa is None:
            self.initializeOS()
        cdef np.ndarray robMove = moveIndices(self.robot.compiledChainsOS,
                                              moveChains, self.robot.OSa.number)
        if objOSa:
            pass
        elif objCC:
            objOSa = compileOSa(objCC, prdebug=prdebug)
        else:
            # This seems to get into trouble with deallocated Numpy arrays??
            if prdebug: print 'DANGER!'
            objOSa = compileOSa([compileObjectFrames(obj, None, False, False, prdebug=prdebug)], prdebug=prdebug)

        if prdebug:
            printOSa(objOSa, 'object')
            raw_input('Object OSa')

        objShapes = {obj.name():obj}
        # Keep attached objects for moving chains
        if moveChains is None:
            attached = self.attachedCCd
            attOSal = self.attOSal
        elif conf.robot.gripperChainNames:
            attached = {h:(a if conf.robot.gripperChainNames.get(h, None) in moveChains else None)\
                        for (h,a) in self.attachedCCd.iteritems()}
            attOSal = [(a if conf.robot.gripperChainNames.get(h, None) in moveChains else None) \
                       for (h, a) in zip(handName, self.attOSal)]
        else:
            attached = {h:None for h in self.robot.handNames()}
            attOSal = [None, None]
        viol, selfColl = confViolationsOS(conf,
                                          (self.robot.compiledChainsOS,
                                           emptyAttachedCCd if ignoreAttached else attached),
                                          self.robot.OSa,
                                          None if ignoreAttached else attOSal,
                                          objOSa,
                                          self.robot.selfCollidePairs if checkSelfCollide else [],
                                          robMove,
                                          objShapes, [obj.name()], prdebug=prdebug)

        if glob.debugRobotCollision:
            print obj
            print obj.toPrims()
            print 'moveChains', moveChains, 'ignoreAttached', ignoreAttached, \
                  'checkSelfCollide', checkSelfCollide
            print '->', viol

        return (viol is None or viol.weight() > 0)

    def draw(self, window, excluded = [],
             drawRobot = True, objectColors = {}, objectOpacities = {},
             drawRegions = False):
        if self.world.workspaceRegion:
            self.world.workspaceRegion.draw(window, color='red')
        colors = ['red','orange','gold', 'green','blue','purple']
        objects = self.getShadowShapes() + self.getNonShadowShapes()
        for place in objects:
            name = place.name()
            if name in excluded: continue
            if name in self.fixedObjects:
                c = 'darkgray'
            else:
                c = objectColors.get(name,
                                 place.properties.get('color', None) \
                                              or random.choice(colors))
            place.draw(window, color = c,
                       opacity=objectOpacities.get(place.name(), 1.0))
        if drawRegions:
            for regName in self.regionShapes:
                self.regionShapes[regName].draw(window, 'purple')
        if drawRobot and self.robotPlace:
            self.robotPlace.draw(window,
                                 color=objectColors.get('robot', 'gold'),
                                 opacity=objectOpacities.get('robot', 1.0))

    def __str__(self):
        return 'WorldState(%s)'%(self.objectConfs)
    __repr__ = __str__

cdef np.ndarray zzTop = np.zeros(0, dtype=np.dtype("i"))

cdef np.ndarray moveIndices(tuple CC, list chains, int number):
    if chains:
        move = np.zeros(number, dtype=np.dtype("i"))
        frames, framesList, allChains, frameChain = CC
        for chain in chains:
            for frame in allChains[chain]:
                for i in frames[frame].osaIndices:
                    move[i] = 1
    else:
        move = zzTop
    return move

# This needs to be placed if object moves, typically just create a new one.
cdef tuple compileObjectFrames(obShape, shShape, fixedObject, fixedShadow, prdebug=False):
    allChains = OrderedDict()
    frameChain = {}
    obj = obShape.name()
    origin = np.ascontiguousarray(obShape.origin().matrix, dtype=np.double)
    frames = {obj : ChainFrameOS(frame=origin,
                                 link=[obShape, shShape],
                                 linkVerts=[linkVerts(obShape, rel=False, prdebug=prdebug),
                                            linkVerts(shShape, rel=False, prdebug=prdebug)] \
                                            if shShape else [linkVerts(obShape, rel=False, prdebug=prdebug)],
                                 bbox=[obShape.bbox(), shShape.bbox()] \
                                       if shShape else [obShape.bbox()],
                                 permanent=[fixedObject, fixedShadow])}
    framesList = [obj]
    allChains[obj] = [obj]
    frameChain[obj] = obj
    cc = frames, framesList, allChains, frameChain
    return cc

def countVerts(CCList, prdebug=False):
    count = 0
    for CC in CCList:
        if prdebug: print 'CC', CC
        frames, framesList, allChains, frameChain = CC
        for chain in allChains:
            if prdebug: print 'chain', chain
            for f in allChains[chain]:
                frame = frames[f]
                if prdebug: print 'f', f, 'frame', frame
                if frame.linkVerts:
                    if len(frame.linkVerts) > 1:
                        if frame.linkVerts[0] is None or frame.linkVerts[1] is None:
                            print f, frame, frame.linkVerts
                            assert None, 'Whoa'
                    assert len(frame.linkVerts) == 1 or len(frame.linkVerts[0]) == len(frame.linkVerts[1])
                    count += len(frame.linkVerts[0])
                    if prdebug: print 'count', count, 'linkVerts', len(frame.linkVerts[0])
    return count

def compileOSa(obCCList, returnObjNames=False, prdebug=False):
    if obCCList is None:
        return OS_Array(0)
    nv = countVerts(obCCList, prdebug)
    osa = OS_Array(nv)
    if prdebug:
        print 'nv', nv, '=', osa.number
    primi = 0
    framei = 0
    objNames = []
    for obCC in obCCList:
        frames, framesList, allChains, frameChain = obCC
        if prdebug:
            print 'frames', frames, 'allChains', allChains
        for chain in allChains:
            for f in allChains[chain]:
                frame = frames[f]
                # An entry for each primitive vertex array
                if frame.linkVerts:
                    osaIndices = []
                    if len(frame.linkVerts) == 2:
                        if len(frame.linkVerts[0]) != len(frame.linkVerts[1]):
                            print frame.linkVerts
                            raw_input('Object parts and shadow parts do not match')
                        for lvo, lvs in zip(frame.linkVerts[0], frame.linkVerts[1]):
                            if prdebug:
                                print 'Setting', [o.name() for o in frame.link], 'perm', frame.permanent
                            verts = [lvo, lvs]
                            if prdebug:
                                print lvo
                                print lvs
                            bboxes = frame.bbox
                            osa.set(primi, frame.frame, verts, bboxes, frame.permanent, [framei, framei+1])
                            osaIndices.append(primi)
                            primi += 1
                        if returnObjNames:
                            objNames.extend([o.name() for o in frame.link])
                        if prdebug:
                            print [framei, framei+1], frame.permanent
                            print 'objNames', objNames, 'osaIndices', osaIndices
                        framei += 2
                    else:
                        for lvo in frame.linkVerts[0]:
                            verts = [lvo]
                            if prdebug:
                                print lvo
                            bboxes = frame.bbox
                            osa.set(primi, frame.frame, verts, bboxes, frame.permanent, [framei])
                            osaIndices.append(primi)
                            primi += 1
                        if returnObjNames:
                            objNames.append(frame.link[0].name())
                        framei += 1
                        if prdebug:
                            print framei, frame.permanent
                            print 'objNames', objNames, 'osaIndices', osaIndices
                    frame.osaIndices = osaIndices
    if returnObjNames:
        return osa, objNames
    else:
        return osa

# This chain will need to be placed for new conf of the robot.
cdef compileAttachedFrames(attached, hand, wristFrameName, attFrame,
                                 fixedHeld, fixedGrasp):
    if (not attached): return None
    attFrame = np.ascontiguousarray(attFrame, dtype=np.double)
    attFrame.flags.writeable = False
    att = list(attached.parts())     # obj, shadow
    # Computes a bbox based on linkVerts radius
    entry = ChainFrameOS(base=wristFrameName,
                         frame=attFrame,
                         link=att, # a list or None
                         linkVerts=[linkVerts(c, rel=True) for c in att],
                         permanent=[fixedHeld, fixedGrasp])
    frames = {'attached': entry}
    return frames
