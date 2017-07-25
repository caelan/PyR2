import platform

class Glob:

    # rrt.py
    useCart = True
    maxStopNodeSteps = 10
    maxRRTIter = 100
    failRRTIter = 10
    rrtInterpolateStepSize = 0.1

    # pr2
    torsoZ = 0.3                            # normally 0.3 or 0.2
    baseGrowthX = 0.1
    baseGrowthY = 0.1
    useOldIKPoses = True

    # genericRobot.py
    cacheGenIK = True
    maxRestartsGenIK = 7

    # objects2.pyx
    useGJK2 = True

    # For approaching a grasp
    approachBackoff = 0.1
    approachPerpBackoff = 0.05

    ########################################
    # Windows
    ########################################
    windowSizes = {'W': 600, 'Belief': 500, 'World': 500, 'MAP': 500}

    # Robot parameters, should be stored and read from robot, here for
    # backward compatibility
    useRight = True
    useLeft = True
    useVertical = True
    useHorizontal = True
    workspace = ((-1.0, -3., 0.0), (3.0, 3., 2.0)) #((-2.5, -2.5, 0.0), (2.5, 2.5, 2.0))

    viewPort = None
    def setViewport(self, workspaceArg):
        ((wx0, wy0, _), (wx1, wy1, wdz)) = workspaceArg
        self.viewPort = [wx0-0.1, wx1+0.1, wy0-0.1, wy1+0.1, 0.0, wdz]

    def usePR2(self, wspace=None, big_workspace=False,
               left=True, right=True, vertical=True, horizontal=True):
        self.useRight = right
        self.useLeft = left
        self.useVertical = vertical
        self.useHorizontal = horizontal
        if wspace:
            self.workspace = wspace
        elif big_workspace:
            self.workspace = ((-2.0, -3., 0.0), (3.0, 3., 2.0))
        else:
            self.workspace = ((-1.0, -3., 0.0), (3.0, 3., 2.0)) #((-2.5, -2.5, 0.0), (2.5, 2.5, 2.0))
        self.setViewport(self.workspace)

    def useIIWA(self, left=True, right=True, vertical=True, horizontal=True,
                big_workspace=False):
        self.useRight = right
        self.useLeft = False            # fixed
        self.useVertical = vertical
        self.useHorizontal = horizontal
        if big_workspace:
            self.workspace = ((-2.0, -3., 0.0), (3.0, 3., 2.0)) #((-2.5, -2.5, 0.0), (2.5, 2.5, 2.0))
        else:
            self.workspace = ((-1.0, -3., 0.0), (3.0, 3., 2.0)) #((-2.5, -2.5, 0.0), (2.5, 2.5, 2.0))
        self.setViewport(self.workspace)

    #############
    # debug flags
    #############

    # rrt.py
    debugRRT = False
    debugRRTDisplay = False
    debugSafeConf = False
    debugRRTFailed = False
    debugVerifyRRTPath = False
    debugFailGoalPath = False
    debugRRTTime = False
    debugBackOff = False

    # pr2InvKin2.pyx
    debugPr2IK = False
    LINUX = (platform.system() == 'Linux')
    IKfastStep = 0.1

    # pr2Robot.py
    debugPr2Head = False

    # mmUtil.py
    debugGenMemoize = False
    debugBigAngleChange = False
    debugObjectGraspFrame = False
    debugRobotGraspFrame = False
    debugConfGD = False

    # collision.pyx
    debugPrimPrimCollides = False

    # cut.pyx
    debugCut = False

    # lazy_vgraph
    debugVgraph = False

    # objects.pyx
    debugRobotCollision = False
    debugAddRegion = False
    debugPlacement = False

    # genericRobot.py
    debugInverseKinGeneric = False

# import this!
glob = Glob()

print 'Loaded amber/globals.py'
