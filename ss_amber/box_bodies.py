import math

import numpy as np
from util.mmUtil import restFaceIndex
from geometry.shapes import BoxAligned, Shape

import geometry.hu as hu

def get_box_body(length, width, height, name='box', color='black'):
  return Shape([
    BoxAligned(np.array([(-length/2., -width/2., 0),
                         (length/2., width/2., height)]), None)
  ], None, name=name, color=color)

def get_box_body_center(length, width, height, name='box', color='black'):
  return Shape([
    BoxAligned(np.array([(-length/2., -width/2., -height/2),
                         (length/2., width/2., height/2)]), None)
  ], None, name=name, color=color)

##################################################

# Grasps

# TODO - rename these
GRASP_FRAME = hu.Pose(0, -0.025, 0, 0)
CENTER_FRAME = hu.Pose(0, 0, 0.025, 0)

def get_grasp_frame_origin(grasp_frame):
  # Find the robot wrist frame corresponding to the grasp at the placement
  objFrame = hu.Pose(0, 0, 0, 0) # objFrame is the origin of the object
  faceFrame = grasp_frame.compose(GRASP_FRAME)
  centerFrame = faceFrame.compose(CENTER_FRAME)
  graspFrame = objFrame.compose(centerFrame)
  return graspFrame

def get_grasp_frame_face(shape, grasp_frame):
  faceFrames = shape.faceFrames()
  # Find the robot wrist frame corresponding to the grasp at the placement
  objFrame = faceFrames[restFaceIndex(shape)].inverse() # objFrame is the origin of the object
  faceFrame = grasp_frame.compose(GRASP_FRAME)
  centerFrame = faceFrame.compose(CENTER_FRAME)
  graspFrame = objFrame.compose(centerFrame)
  return graspFrame

##########

# from the side
# the z offset raises or lowers the grasp relative to midpoint of object
gMat0 = hu.Transform(np.array([(0.,1.,0.,0.),
                               (0.,0.,1.,-0.025),
                               (1.,0.,0.,0.02),
                               (0.,0.,0.,1.)]))
#gMat0h = hu.Transform(np.array([(0.,1.,0.,0.0),
#                               (0.,0.,1.,-0.025),
#                               (1.,0.,0.,0.05),
#                               (0.,0.,0.,1.)]))
gMat1 = hu.Transform(np.array([(0.,-1.,0.,0.),
                               (0.,0.,-1.,0.025),
                               (1.,0.,0.,0.02),
                               (0.,0.,0.,1.)]))
#gMat1h = hu.Transform(np.array([(0.,-1.,0.,0.),
#                               (0.,0.,-1.,0.025),
#                               (1.,0.,0.,0.05),
#                               (0.,0.,0.,1.)]))
gMat4 = hu.Pose(0,0,0,math.pi/2).compose(gMat0)
gMat5 = hu.Pose(0,0,0,-math.pi/2).compose(gMat0)

SIDE_GRASPS = [gMat0, gMat1, gMat4, gMat5]

def get_box_side_grasps(shape):
  return [get_grasp_frame_face(shape, grasp_frame) for grasp_frame in SIDE_GRASPS]

##########

# from the top
gMat2 = hu.Transform(np.array([(-1.,0.,0.,0.),
                               (0.,0.,-1.,0.025),
                               (0.,-1.,0.,0.),
                               (0.,0.,0.,1.)]))
gMat3 = hu.Transform(np.array([(1.,0.,0.,0.),
                               (0.,0.,1.,-0.025),
                               (0.,-1.,0.,0.),
                               (0.,0.,0.,1.)]))

TOP_GRASPS = [gMat2, gMat3]

def get_box_top_grasps(shape):
  return [get_grasp_frame_face(shape, grasp_frame) for grasp_frame in TOP_GRASPS]
