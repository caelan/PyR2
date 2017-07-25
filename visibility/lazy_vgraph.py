
# Based on code originally written by Caelan Garrett for 6.s078
# Modified substantially by Tomas Lozano-Perez

import pdb
from time import clock
from heapq import heappush, heappop

from geometry.shapes import convexHullVertsXY
import numpy as np
from geometry.hu import angleDiff
from scipy.spatial import cKDTree

import graphics.windowManager3D as wm
from visibility.caelan_geometry import *
import hu
from autil.globals import glob

window = None

def zoverlap(s1, s2):
    zr1 = s1.zRange(); zr2 = s2.zRange()
    return not (zr1[0] > zr2[1] or zr1[1] < zr2[0])

def object_from_shape(shape, origin):
    points = convexHullVertsXY(shape.getVertices())
    return Object(origin, [Polygon([Point(points[0,i], points[1,i]) \
                                    for i in range(points.shape[1])])])

def xyCO(A, B, signs=(-1,1)):
    Averts = A.vertices()
    Bverts = B.vertices()
    verts = []
    for i in range(Bverts.shape[1]):
        verts.append(signs[1]*Bverts[:,i:i+1] + signs[0]*Averts)
    for v in verts: v[3,:]=1.0
    zrA = [signs[0]*x for x in A.zRange()]
    zrB = [signs[1]*x for x in B.zRange()]
    zr = min(zrB[0]+zrA[0],zrB[0]+zrA[1]),max(zrB[1]+zrA[0],zrB[1]+zrA[1])

    return convexHullVertsXY(np.hstack(verts))

def xyCOParts(A, B, signs=(-1,1)):
    co = []
    for a in A.parts():
        for b in B.parts():
            if zoverlap(a, b):
                co.append(xyCO(a,b,signs))
    return [Polygon([Point(points[0,i], points[1,i]) \
                     for i in range(points.shape[1])])
            for points in co]

class World:
  def __init__(self, xr, yr, robot, obstacles, angle_slices, cspace_obstacles={}, thConvert=None):
      self.xr = xr
      self.yr = yr
      self.robot = robot
      self.obstacles = obstacles
      self.region = AABB(Point(xr[0], yr[0]), Point(xr[1], yr[1]))
      self.angle_slices = angle_slices
      self.cspace_obstacles = cspace_obstacles
      self.verts = set([])              # Poses
      self.neighbors = {}               # Pose: list of poses
      self.edges = {}                   # (Pose1, Pose2): cost
      self.thConvert = thConvert or self.robot.get_radius()

  def set_start_and_goal(self, start, goal):
      if not self.valid_pose(start):
          if glob.debugVgraph:
              print 'start pose is not valid'
              pdb.set_trace()
              return False
      if not self.valid_pose(goal):
          if glob.debugVgraph:
              print 'goal pose is not valid'
              pdb.set_trace()
              return False
      for orientation in self.angle_slices:
          for x in (start, goal):
              assert not x.position.source
              self.verts.add(Pose(x.position, orientation))
      return True

  def valid_pose(self, pose):
      return pose.position.x <= self.xr[1] and pose.position.x >= self.xr[0] and \
             pose.position.y <= self.yr[1] and pose.position.y >= self.yr[0] and \
             pose.orientation in self.angle_slices and \
             not self.collide(pose.position, self.get_cspace_obstacles(pose.orientation))

  def get_cspace_obstacles(self, orientation):
      if orientation not in self.cspace_obstacles:
          self.cspace_obstacles[orientation] = [] 
      for obst in [obst.cspace_object(self.robot.rotate(orientation)) for obst in self.obstacles]:
          self.cspace_obstacles[orientation].extend(obst.polys)
      return self.cspace_obstacles[orientation]

  def make_verts(self):
      for orientation in self.angle_slices:
          for poly in self.cspace_obstacles[orientation]:
              for pt in poly.points:
                  if not(pt.x <= self.xr[1] and pt.x >= self.xr[0] and \
                         pt.y <= self.yr[1] and pt.y >= self.yr[0]):
                      continue
                  # All the points go into all the orientations
                  for orientation2 in self.angle_slices:
                      other_poly = [poly2 for poly2 in self.cspace_obstacles[orientation2] \
                                    if poly is not poly2]
                      if not self.collide(pt, other_poly):
                          self.verts.add(Pose(pt, orientation2))

  # Could later add support to ensure that actions happen in a region
  def collide(self, point, cspace_obstacles):
      return any(poly.contains(point) for poly in cspace_obstacles)

  # cspace_obstacles are already oriented for the correct slice
  # ignore lines that are not tangential, that is, extending the line causes collision.
  def translation_collide(self, start, end, cspace_obstacles): 
      if start.source == end.source:
          movement = Line(start, end)
          return any([poly.collides(movement) for poly in cspace_obstacles if poly != start.source])
      elif start.source and end.source:
          sources = (start.source, end.source)
          movement = Line(start, end)
          return start.source.collides(movement) or end.source.collides(movement) or \
                 any(poly.collides(movement) for poly in cspace_obstacles if poly not in sources)
      elif start.source:
          # movement = Line(2*start - end, end)
          movement = Line(start, end)
          # not outer(start.source, start, end) or \
          return start.source.collides(movement) or \
                 any(poly.collides(movement) for poly in cspace_obstacles if poly != start.source)
      elif end.source:
          # movement = Line(start, 2*end - start)
          movement = Line(end, start)
          # not outer(end.source, end, start) or \
          return end.source.collides(movement) or \
                 any(poly.collides(movement) for poly in cspace_obstacles if poly != end.source)
      else:
          return any(poly.collides(movement) for poly in cspace_obstacles)

  def rotation_collide(self, point, cspace_obstacles_1, cspace_obstacles_2): 
      return self.collide(point, cspace_obstacles_1) or self.collide(point, cspace_obstacles_2)

  def collision_test(self, x, y, cross=False):
      if x.orientation == y.orientation:
          return self.translation_collide(x.position, y.position, self.cspace_obstacles[x.orientation])
      elif x.position == y.position:
          return self.rotation_collide(x.position,
                                       self.cspace_obstacles[x.orientation], self.cspace_obstacles[y.orientation])
      elif cross:
          return self.translation_collide(x.position, y.position, self.cspace_obstacles[x.orientation]) \
                 or self.translation_collide(x.position, y.position, self.cspace_obstacles[y.orientation])
      else:                             # Don't allow edges across orientations
          return True


  def remove_vertex(self, v):
      # Remove vertex from all orientations
      for orientation in self.angle_slices:
          v2 = Pose(v.position, orientation)
          if v2 in self.verts:
              self.verts.remove(v2)
          if v2 in self.neighbors:
              del self.neighbors[v2]
          for vn in self.neighbors.values():
              if v2 in vn:
                  vn.remove(v2)

  def add_vertex(self, v):
      for orientation in self.angle_slices:
          v2 = Pose(v.position, orientation)
          if v2 in self.neighbors:
              v2n = self.neighbors[v2]
          else:
              v2n = set([])
              self.neighbors[v2] = v2n
          self.verts.add(v2)
          for vo in self.verts:
              if vo == v2: continue
              if not (vo.position == v2.position or vo.orientation == v2.orientation):
                  continue
              if v2 in self.neighbors[vo]:
                  v2n.add(vo)
              elif not self.collision_test(vo, v2):
                  v2n.add(vo)
                  self.neighbors[vo].add(v2)

  def build_vgraph(self):
      self.make_verts()
      for v in self.verts:
          self.neighbors[v] = set([])
      for v in self.verts.copy():
          self.add_vertex(v)

  def get_neighbors_all(self, pose):
      return self.neighbors.get(pose, [])

  def draw_neighbors(self):
      colors = ('gray', 'orange')
      for v in self.neighbors:
          for i, angle in enumerate(self.angle_slices):
              for v2 in self.neighbors[v]:
                  if v2.orientation == angle:
                      Line(v.position, v2.position).draw(window, color = colors[i], width = 2)

  def draw_visibility_graph(self):
      for start, end in self.edges:
          if start.position != end.position and self.edges[(start, end)]:
              Line(start.position, end.position).draw(window, color = 'blue', width = 2)

  def get_neighbors(self, pose, expand=True, special=[]):
      if pose in special: expand = True
      edges = self.edges
      for v in self.verts:
          if v != pose:
              # if we don't know it's blocked
              d = self.edges.get((v, pose), None) or self.edges.get((pose, v), None)
              if d is None and (expand or v in special):  # if expand is False, unknown means False
                  if self.collision_test(v, pose):
                      d = edges[(v, pose)] = False
                  else:
                      d = edges[(v, pose)] = True
              if d is True: yield v

  # Admissible heuristic
  def distance(self, one, two):
      return (two.position - one.position).length() + self.thConvert*abs(angleDiff(two.orientation, one.orientation))

  # Inadmissible heuristic
  def fast_heuristic(self, one, two):
      return 10*self.distance(one, two)

  def retrace_weighted_path(self, path_map, node):
      success = []
      retrace = node
      while retrace != None:
          success.append(retrace)
          retrace = path_map[retrace][1]
      success.reverse()
      return success, path_map[node][0] # path, dist

  def astar(self, start, goal, heuristic, expand=True):
      paths = [(heuristic(start, goal), 0, start)]
      visited = {}
      visited[start] = (0, None)
      while len(paths) != 0:
          (h_dist, dist, last) = heappop(paths)
          if visited[last][0] != dist: #Becuase I'm not using a fibonacci heap, bad paths may be in the queue
              continue
          if last == goal:
              if glob.debugVgraph:
                  print 'astar succeeded with expand=', expand
              return self.retrace_weighted_path(visited, last)
          for next in self.get_neighbors(last, expand=expand, special=(start, goal)):
              new_dist = dist + self.distance(last, next)
              if (next not in visited or new_dist < visited[next][0]):
                  heappush(paths, (new_dist + heuristic(next, goal), new_dist, next)) #Substitute for decrease_key
                  visited[next] = (new_dist, last)
      if glob.debugVgraph:
          print 'astar failed with expand=', expand
      return None, None

  def astar_all(self, start, goal, heuristic):
      paths = [(heuristic(start, goal), 0, start)]
      visited = {}
      visited[start] = (0, None)
      while len(paths) != 0:
          (h_dist, dist, last) = heappop(paths)
          if visited[last][0] != dist: #Becuase I'm not using a fibonacci heap, bad paths may be in the queue
              continue
          if last == goal:
              if glob.debugVgraph:
                  print 'astar succeeded with expand=', expand
              return self.retrace_weighted_path(visited, last)
          for next in self.get_neighbors_all(last):
              new_dist = dist + self.distance(last, next)
              if (next not in visited or new_dist < visited[next][0]):
                  heappush(paths, (new_dist + heuristic(next, goal), new_dist, next)) #Substitute for decrease_key
                  visited[next] = (new_dist, last)
      if glob.debugVgraph:
          print 'astar failed with expand=', expand
      return None, None

  def drawRobotPath(self, path, color = None):
      for i in range(len(path) - 1):
          Line(path[i].position, path[i+1].position).draw(window, color = color, width = 3)
          self.robot.at_pose(path[i+1]).draw(window, color)

  def drawPath(self, path, color = None):
      for i in range(len(path) - 1):
          Line(path[i].position, path[i+1].position).draw(window, color = color, width = 3)

  # Search path with pre-computed Vgraph
  def search_all(self, start, goal, cached=False, display=False, storePaths=True):
      if not self.collision_test(start, goal, cross=True):
          if display: print 'Simple connect!'
          path, distance = [start, goal], self.distance(start, goal)
      else:
          if not self.verts:
              t1 = clock()
              self.build_vgraph()
              if glob.debugVgraph: print 'Building vgraph took', clock() - t1
              t1 = clock()
              self.add_vertex(goal)
              connect_to_goal(self, goal)
              if glob.debugVgraph: print 'Finding paths took', clock() - t1
                            
          if self.collide(start.position, self.cspace_obstacles[start.orientation]):
              return None, None
          if storePaths:
              distance = query_dist_to_goal(self, start)
              path = distance
          else:
              self.add_vertex(start)
              self.add_vertex(goal)
              path, distance = self.astar_all(start, goal, self.distance)
              self.remove_vertex(start)     # assume goal will get re-used
              # Verify the query
              d = query_dist_to_goal(self, start)
              if abs(d-distance) > 0.1*distance:
                  if glob.debugVgraph:
                      print 'bad distance estimate', 'distance', distance, 'd', d
                      for x in path: print '**', x
                      d = query_dist_to_goal(self, start, compare=distance)
      if not display:
          return path, distance

      for orientation,color in zip(self.cspace_obstacles, ('gray', 'orange')):
          for obs in self.cspace_obstacles[orientation]:
              obs.draw(window, color)
      self.draw_neighbors()
      start.draw(window, color='red', radius=3)
      goal.draw(window, color='gold', radius=3)
      if path is None:
          print 'No path to goal'
          pdb.set_trace()
      else:
          print 'Path has length ', distance
          self.drawPath(path, color = 'purple')
          self.drawRobotPath(path, color='green')
          raw_input('Go?')
      window.update()
      return path, distance

  # Search path in incrementally-built Vgraph
  def search(self, start, goal, cached=False, display=False):
      if not self.collision_test(start, goal, cross=True):
          if display: print 'Simple connect!'
          path, distance = [start, goal], self.distance(start, goal)
      else:
          t1 = clock()
          if not cached:
              self.make_verts()
          if self.set_start_and_goal(start, goal):
              path, distance = self.astar(start, goal, self.distance, expand=(False if cached else True))
              if cached and not path:       # Try again with expansion
                  path, distance = self.astar(start, goal, self.distance, expand=True)
              if glob.debugVgraph: print 'Search took ', clock() - t1, ' seconds'
          else:
              path = distance = None

      if not display:
          return path, distance

      for orientation,color in zip(self.cspace_obstacles, ('gray', 'orange')):
          for obs in self.cspace_obstacles[orientation]:
              obs.draw(window, color)
      self.draw_visibility_graph()
      start.draw(window, color='red', radius=3)
      goal.draw(window, color='gold', radius=3)
      if path is None:
          print 'No path to goal'
          pdb.set_trace()
      else:
          print 'Path has length ', distance
          self.drawPath(path, color = 'purple')
          self.drawRobotPath(path, color='green')
          raw_input('Go?')
      window.update()
      return path, distance


def outer(poly, p1, p2):
    if not poly.line_ids:
        poly.get_lines()                # make sure line_ids are there
    for (n,d) in poly.line_ids[p1]:
        if n.dot(p2) + d < 0: return False
    return True

# Do setup for incremental Vgraph
def basePath(pbs, prob, basePose, goalPose, display=False, thConvert=1.0):
    # t0 = clock()
    basePose = basePose.pose()
    goalPose = goalPose.pose()
    if goalPose.theta != basePose.theta and abs(goalPose.theta - basePose.theta) < 0.01:
        # make the theta's the same
        goalPose = hu.Pose(goalPose.x, goalPose.y, 0, basePose.theta)
    shape = pbs.getRobot().baseLinkShape()
    baseShape = shape.applyTrans(hu.Pose(0,0,0,basePose.theta))
    goalShape = shape.applyTrans(hu.Pose(0,0,0,goalPose.theta))
    shWorld = pbs.getShadowWorld(prob)
    cspace_obstacles = {basePose.theta: [],
                        goalPose.theta: []}
    obstacles = []
    for perm in shWorld.fixedObjects:
        obst = shWorld.objectShapes[perm]
        if 'shadow' in obst.name(): continue
        if zoverlap(baseShape, obst):
            obstacles.append(obst)
    start = Pose(Point(basePose.x, basePose.y), basePose.theta)
    goal = Pose(Point(goalPose.x, goalPose.y), goalPose.theta)

    key = (frozenset(obstacles), start.orientation, goal.orientation)
    cached, world = pbs.genCacheQuery('vgraph', key)
    if not cached:
        for perm in shWorld.fixedObjects:
            obst = shWorld.objectShapes[perm]
            if 'shadow' in obst.name(): continue
            if zoverlap(baseShape, obst):
                cspace_obstacles[basePose.theta].extend(xyCOParts(baseShape, obst))
                if goalPose.theta != basePose.theta:
                    cspace_obstacles[goalPose.theta].extend(xyCOParts(goalShape, obst))
        robot = object_from_shape(shape, Point(0,0))
        ((x0, y0, z0), (x1, y1, z0)) = pbs.getWorld().workspace
        world = World((x0, x1), (y0, y1), robot, [],
                      cspace_obstacles.keys(), cspace_obstacles, thConvert=thConvert)
        pbs.genCacheSet('vgraph', key, world)
        cached = False

    global window
    if display:
        if not window:
            if 'VG' in wm.windows:
                window = wm.windows['VG']
            else:
                ((x0, y0, z0), (x1, y1, z0)) = pbs.getWorld().workspace
                window = wm.makeDrawingWindow(600, 600, x0, x1, y0, y1, 'VG')
        else:
            window.clear()
    ans = world.search(start, goal, cached=cached, display=display)
    # print 'basePath took ', clock() - t0, ' seconds', 'cached=', cached
    return ans

# Set up world with pre-computed Vgraph
def setupBasePathWorld(pbs, prob, basePose, goalPose, display=False, thConvert=1.0):
    # t0 = clock()
    basePose = basePose.pose()
    goalPose = goalPose.pose()
    if goalPose.theta != basePose.theta and abs(goalPose.theta - basePose.theta) < 0.01:
        # make the theta's the same
        goalPose = hu.Pose(goalPose.x, goalPose.y, 0, basePose.theta)
    shape = pbs.getRobot().baseLinkShape()
    baseShape = shape.applyTrans(hu.Pose(0,0,0,basePose.theta))
    goalShape = shape.applyTrans(hu.Pose(0,0,0,goalPose.theta))
    shWorld = pbs.getShadowWorld(prob)
    cspace_obstacles = {basePose.theta: [],
                        goalPose.theta: []}
    obstacles = list(pbs.getWorld().workspaceWalls)
    for perm in shWorld.fixedObjects:
        obst = shWorld.objectShapes[perm]
        if 'shadow' in obst.name(): continue
        if zoverlap(baseShape, obst):
            obstacles.append(obst)
    key = (frozenset(obstacles), basePose.theta, goalPose.theta)
    cached, world = pbs.genCacheQuery('vgraph_all', key)
    if not cached:
        for perm in shWorld.fixedObjects:
            obst = shWorld.objectShapes[perm]
            if 'shadow' in obst.name(): continue
            if zoverlap(baseShape, obst):
                cspace_obstacles[basePose.theta].extend(xyCOParts(baseShape, obst))
                if goalPose.theta != basePose.theta:
                    cspace_obstacles[goalPose.theta].extend(xyCOParts(goalShape, obst))
        robot = object_from_shape(shape, Point(0,0))
        ((x0, y0, z0), (x1, y1, z1)) = pbs.getWorld().workspace
        world = World((x0, x1), (y0, y1), robot, [],
                      cspace_obstacles.keys(), cspace_obstacles, thConvert=thConvert)
        pbs.genCacheSet('vgraph_all', key, world)

    global window
    if display:
        if not window:
            if 'VG' in wm.windows:
                window = wm.windows['VG']
            else:
                ((x0, y0, z0), (x1, y1, z0)) = pbs.getWorld().workspace
                window = wm.makeDrawingWindow(600, 600, x0, x1, y0, y1, 'VG')
        else:
            window.clear()

    return world

# Find all distances from vgraph vertices (poses) to goal pose
def connect_to_goal(world, goal):
    dist = world.distance
    neigh = world.neighbors
    G = {v:{v2:dist(v, v2) for v2 in neigh[v]} for v in neigh if neigh[v]}
    D, P = Dijkstra(G, goal)
    world.goal_dists = D
    world.pred = P
    pts = list(set([v.position for v in G]))
    data = np.zeros((len(pts), 2))
    for i,p in enumerate(pts):
        data[i,0] = p.x; data[i,1] = p.y
    tree = cKDTree(data, 10)
    world.points = pts
    world.kdtree = tree

# find N closest neighbors of pt and find the one that gets to goal quickest.
query_neighbors = 30                    # N
def query_dist_to_goal(world, pt, display=False, compare=None):
    def tracePath(qpt):
        print 'distance', answers[0][0]
        while qpt is not None:
            print qpt, world.goal_dists[qpt]
            qpt = world.pred.get(qpt, None)
    i = 0
    tree = world.kdtree
    points = world.points
    pos = {orientation: Pose(pt.position, orientation) \
           for orientation in world.angle_slices}
    q = np.array([[pt.position.x, pt.position.y]])
    assert q.shape == (1,2)
    answers = []
    angles = [th for th in world.angle_slices \
              if not world.collide(pt.position, world.cspace_obstacles[th])]
    for k in range(min(len(points), query_neighbors+10), len(points)+1, 10):
        dists, ids = tree.query(q, k)
        for j in range(i, k):
            for orientation in angles:
                qpt = Pose(points[ids[0,j]], orientation)
                if not qpt in world.goal_dists: continue
                if not world.collision_test(pos[orientation], qpt):
                    qd = dists[0,j] + world.goal_dists[qpt] + \
                         world.thConvert*abs(angleDiff(pt.orientation, orientation))
                    answers.append((qd, qpt))
        if len(answers) >= query_neighbors: break
        i = k
    if not answers: return None
    answers.sort()
    qd = answers[0][0]
    if display or compare:
        print 'start', pt
        tracePath(answers[0][1])
        pdb.set_trace()
    return qd

def checkAngles(angle_slices, angle):
    if angle in angle_slices: return angle
    best_d = 1000.
    best_a = None
    for a in angle_slices:
        d = abs(angleDiff(a,angle))
        if d < best_d:
            best_d = d
            best_a = a
    return best_a
    assert None, 'Unknown angle'

# Used in heuristic for moveLookPath
def findBasePath(world, baseConf, goalConf, display=False):
    baseTheta = checkAngles(world.angle_slices, baseConf[-1])
    start = Pose(Point(baseConf[0], baseConf[1]), baseTheta)
    goalTheta = checkAngles(world.angle_slices, goalConf[-1])
    goal = Pose(Point(goalConf[0], goalConf[1]), goalTheta)
    if start == goal: return [start], 0
    ans = world.search_all(start, goal, display=display)
    return ans

# Dijkstra's algorithm for shortest paths
# David Eppstein, UC Irvine, 4 April 2002

# http://aspn.activestate.com/ASPN/Cookbook/Python/Recipe/117228
from priodict import priorityDictionary

def Dijkstra(G,start,end=None):
    """
    Find shortest paths from the start vertex to all
    vertices nearer than or equal to the end.

    The input graph G is assumed to have the following
    representation: A vertex can be any object that can
    be used as an index into a dictionary.  G is a
    dictionary, indexed by vertices.  For any vertex v,
    G[v] is itself a dictionary, indexed by the neighbors
    of v.  For any edge v->w, G[v][w] is the length of
    the edge.  This is related to the representation in
    <http://www.python.org/doc/essays/graphs.html>
    where Guido van Rossum suggests representing graphs
    as dictionaries mapping vertices to lists of neighbors,
    however dictionaries of edges have many advantages
    over lists: they can store extra information (here,
    the lengths), they support fast existence tests,
    and they allow easy modification of the graph by edge
    insertion and removal.  Such modifications are not
    needed here but are important in other graph algorithms.
    Since dictionaries obey iterator protocol, a graph
    represented as described here could be handed without
    modification to an algorithm using Guido's representation.

    Of course, G and G[v] need not be Python dict objects;
    they can be any other object that obeys dict protocol,
    for instance a wrapper in which vertices are URLs
    and a call to G[v] loads the web page and finds its links.
    
    The output is a pair (D,P) where D[v] is the distance
    from start to v and P[v] is the predecessor of v along
    the shortest path from s to v.
    
    Dijkstra's algorithm is only guaranteed to work correctly
    when all edge lengths are positive. This code does not
    verify this property for all edges (only the edges seen
    before the end vertex is reached), but will correctly
    compute shortest paths even for some graphs with negative
    edges, and will raise an exception if it discovers that
    a negative edge has caused it to make a mistake.
    """

    D = {}  # dictionary of final distances
    P = {}  # dictionary of predecessors
    Q = priorityDictionary()   # est.dist. of non-final vert.
    Q[start] = 0
    
    for v in Q:
        D[v] = Q[v]
        if v == end: break
        
        for w in G[v]:
            vwLength = D[v] + G[v][w]
            if w in D:
                if vwLength < D[w]:
                    raise ValueError, \
  "Dijkstra: found better path to already-final vertex"
            elif w not in Q or vwLength < Q[w]:
                Q[w] = vwLength
                P[w] = v
    
    return (D,P)
