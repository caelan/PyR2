from itertools import product
from random import sample
import math

import numpy as np

from geometry import hu as hu
from ss_amber.box_bodies import get_box_body
from ss_amber.utils import set_pose

class ManipulationProblem:
    def __init__(self, object_names=[], table_names=[], initial_poses={},
                 goal_poses={}, known_poses=[]):
        self.object_names = object_names
        self.table_names = table_names
        self.initial_poses = initial_poses
        self.goal_poses = goal_poses
        self.known_poses = known_poses

def dantam2(world_state): # (Incremental Task and Motion Planning: A Constraint-Based Approach)
  m, n = 3, 3
  #n_obj = 8
  n_obj = 2
  side_dim = .07 # .05 | .07
  height_dim = .1
  box_dims = (side_dim, side_dim, height_dim)
  separation = (side_dim, side_dim)

  coordinates = list(product(range(m), range(n)))
  assert n_obj <= len(coordinates)
  obj_coordinates = sample(coordinates, n_obj)

  length = m*(box_dims[0] + separation[0])
  width = n*(box_dims[1] + separation[1])
  height = .7
  table = get_box_body(length, width, height, name='table', color='brown')
  world_state.world.addObjectShape(table)
  set_pose(world_state, table, hu.Pose(0, 0, 0, 0))

  robot = world_state.robot
  #base_conf = world_state.robot.nominalConf.baseConf() # (0, 0, 0)
  base_conf = (-.75, .2, -math.pi/2)
  initial_conf = robot.makeConf(*base_conf, ignoreStandard=True)
  for arm in robot.gripperChainNames:
    initial_conf.set(robot.gripperChainNames[arm], [robot.gripMax])
  world_state.setRobotConf(initial_conf)
  #world_state.setRobotConf(world_state.robotConf.setBaseConf(baseConf))

  BODY_PLACEMENT_Z_OFFSET = 1e-3
  poses = []
  #z = height + height_dim/2 + BODY_PLACEMENT_Z_OFFSET
  z = height + BODY_PLACEMENT_Z_OFFSET
  theta = 0
  for r in range(m):
    row = []
    x = -length/2 + (r+.5)*(box_dims[0] + separation[0])
    for c in range(n):
      y = -width/2 + (c+.5)*(box_dims[1] + separation[1])
      row.append(hu.Pose(x, y, z, theta))
    poses.append(row)

  initial_poses = {}
  goal_poses = {}
  # TODO - randomly assign goal poses here
  for i, (r, c) in enumerate(obj_coordinates):
    row_color = np.zeros(4)
    row_color[2-r] = 1.
    if i == 0:
      name = 'goal%d-%d'%(r, c)
      color = 'blue'
      goal_poses[name] = poses[m/2][n/2]
    else:
      name = 'block%d-%d'%(r, c)
      color = 'red'
    initial_poses[name] = poses[r][c]
    obj = get_box_body(*box_dims, name=name, color=color)
    world_state.world.addObjectShape(obj)
    #print hash(poses[r][c])
    set_pose(world_state, obj, poses[r][c])

  known_poses = [pose for col in poses for pose in col]

  return ManipulationProblem(
      object_names=initial_poses.keys(), table_names=[table.name()],
      goal_poses=goal_poses,
      initial_poses=initial_poses, known_poses=known_poses)