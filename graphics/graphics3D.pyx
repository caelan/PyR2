import time
import numpy as np
cimport numpy as np
from cpython cimport bool
import graphics.DrawingWindowStandalonePIL as dw
import geometry.hu as hu

cdef class Window3D:
    def __init__(self, viewport = None, str title = 'View', int windowWidth = 500, noWindow = False):

        if noWindow:
            self.window = None
            return

        self.capture = []
        self.capturing = False
        self.modified = False

        cdef float lox, hix, loy, hiy, loz, hiz
        (lox, hix, loy, hiy, loz, hiz) = viewport
        windowHeight = (hiy + hiz - loy - loz) * (float(windowWidth)/(hix-lox))
        self.xzOffset = (0, hiy)
        self.window = dw.DrawingWindow(windowWidth, windowHeight, lox, hix, loy,
                                       hiy + hiz - loz, title)

    def clear(self):
        if self.window is None: return
        self.window.clear()
        if self.capturing:
            if not self.capture or not self.capture[-1][0] == 'clear':
                self.capture.append(['clear', time.clock()])

    def pause(self):
        if self.window is None: return
        if self.capturing:
            if not self.capture or not self.capture[-1][0] == 'pause':
                self.capture.append(['pause', time.clock()])

    def startCapture(self):
        if self.window is None: return
        self.capture = []
        self.capturing = True

    def stopCapture(self):
        if self.window is None: return
        self.capturing = False
        return self.capture

    def playback(self, capture = None, delay = 0, update=False):
        if self.window is None: return
        capturing = self.capturing
        self.capturing = False
        for entry in capture or self.capture:
            if entry[0] == 'clear':
                self.clear()
                time.sleep(delay)
            elif entry[0] == 'pause':
                self.update()
                time.sleep(delay)
            else:
                self.draw(entry[0], color=entry[1], opacity=entry[2])
                if update: self.update()
        self.capturing = capturing

    cpdef draw(self, thing, str color = None, float opacity = 1.0):
        if self.window is None: return
        cdef:
            # shapes.Prim prim
            np.ndarray[np.float64_t, ndim=2] verts, matrix
            np.ndarray[np.float64_t, ndim=1] p1, p2
            np.ndarray[np.int_t, ndim=2] edges
            float dx, dy
            int i, n, e
        if self.capturing:
            self.capture.append([thing,
                                 color,
                                 opacity,
                                 time.clock()])
        (dx, dy) = self.xzOffset
        if isinstance(thing, hu.Point):
            matrix = thing.matrix
            self.window.Point(matrix[0,0], matrix[1,0], color = color or 'black')
            self.window.drawPoint(matrix[0,0]+dx, matrix[2,0]+dy, color = color or 'black')
        elif isinstance(thing, np.ndarray):
            for i in range(thing.shape[1]):
                matrix = thing
                self.window.drawPoint(matrix[0,i], matrix[1,i], color = color or 'black')
                self.window.drawPoint(matrix[0,i]+dx, matrix[2,i]+dy, color = color or 'black')
        elif hasattr(thing, 'parts'):
            for part in thing.parts():
                partColor = color or part.properties.get('color', None) or 'black'
                # if isinstance(part, shapes.Shape):
                if part.__class__.__name__ == 'Shape':
                    for p in part.parts(): self.draw(p, color=partColor, opacity=opacity)
                # elif isinstance(part, (shapes.Thing, shapes.Prim)):
                # elif part.__class__.__name__ in ('Thing', 'Prim'):
                elif hasattr(part, 'vertices') and hasattr(part, 'edges'):
                    verts = part.vertices()
                    edges = part.edges()
                    for e in range(edges.shape[0]):
                        p1 = verts[:,edges[e,0]]
                        p2 = verts[:,edges[e,1]]
                        self.window.drawLineSeg(p1[0], p1[1], p2[0], p2[1],
                                                color = partColor)
                        self.window.drawLineSeg(p1[0]+dx, p1[2]+dy, p2[0]+dx, p2[2]+dy,
                                                color = partColor)

    cpdef update(self):
        if self.window is None: return
        self.window.canvas.update()
