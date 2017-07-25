import graphics.DrawingWindowStandalonePIL as dw
# Without PIL, the graphics in the gen logs doesn't work, but everything else does.
# import DrawingWindowStandalone as dw
reload(dw)
from graphics.graphics3D import Window3D

######################################################################
##  Window management
######################################################################

use3D = False
windows = {}

def forgetWindows():
    windows = {}

def addWindow(w, title):
    windows[title] = w

def makeWindow(title, viewPort = (-1.,1., -1.,1., 0.,1.), windowWidth = 500, noWindow = False):
    if not title in windows:
        if use3D:
            # windows[title] = om.VisualWindow(title = title,
            #                      windowDims = (windowWidth, windowWidth))
            pass
        else:
            #print viewPort
            windows[title] = Window3D(title = title,
                                      windowWidth = windowWidth,
                                      noWindow = noWindow,
                                      viewport = viewPort)
    return windows[title]

def getWindow(w):
    if isinstance(w, str):
        return makeWindow(w)
    else:
        return w

def makeSimpleWindow(dx, dy, windowWidth, title):
    if not title in windows:
        windows[title] = dw.DrawingWindow(windowWidth, windowWidth,
                                          0, max(dx, dy), 0, max(dx, dy), 
                                          title)
    return windows[title]


def makeDrawingWindow(windowWidth, windowHeight, xMin, xMax, yMin, yMax,
                      title):
    if not title in windows:
        windows[title] = dw.DrawingWindow(windowWidth, windowHeight,
                                          xMin, xMax, yMin, yMax,
                                          title)
    return windows[title]
