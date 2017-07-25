from hpn.traceFile import traceStart, traceEnd, trAlways, tr
import numpy as np
import geometry.hu as hu
import time
import geometry.shapes as shapes

Ident = hu.Transform(np.eye(4, dtype=np.float64)) # identity transform

def Ba(bb, **prop): return shapes.BoxAligned(np.array(bb), Ident, **prop)
def Sh(*args, **prop): return shapes.Shape(list(args), Ident, **prop)

def predefined_links(param):
	if param == 'iiwaRightArmLinks':
		return iiwaArmLinks()
	elif param == 'iiwaHeadLinks':
		return iiwaHeadLinks()
	elif param == 'iiwaBaseLink':
		return Sh(Ba(((-0.113, -0.103822, 0.), (0.103759, 0.103822, 0.5)),
			     name='iiwa_base'), name='iiwa_base')
	assert None, 'Unknown links'

'''
dir = "directory containing link_i.off files"
for i in range(8): print readOff(dir+"link_%d.off"%i, name="link"+str(i))
-- IIWA7
link0:((-0.113, -0.103822, -4.19605e-07), (0.103759, 0.103822, 0.1575))
link1:((-0.06798, -0.114593, -0.01), (0.0679801, 0.0679906, 0.250993))
link2:((-0.067994, -0.0679981, -0.0679986), (0.067994, 0.184, 0.114408))
link3:((-0.068, -0.068, -0.00999998), (0.068, 0.114593, 0.283494))
link4:((-0.067994, -0.0679981, -0.0679986), (0.067994, 0.184, 0.114408))
link5:((-0.068, -0.068, -0.01), (0.068, 0.0990203, 0.266793))
link6:((-0.0661488, -0.0863605, -0.0105), (0.0661525, 0.0909, 0.126909))
link7:((-0.0519041, -0.0518698, -2.58205e-06), (0.0519314, 0.0519428, 0.0449975))
-- IIWA14
link0:((-0.136, -0.121384, -4.321e-07), (0.121243, 0.121384, 0.1575))
link1:((-0.0854912, -0.115686, -0.01), (0.0854912, 0.0854874, 0.288))
link2:((-0.0850566, -0.0854325, -0.0679994), (0.085929, 0.204501, 0.132107))
link3:((-0.068, -0.068, -0.01), (0.0679997, 0.114593, 0.283494))
link4:((-0.0679985, -0.0679989, -0.0679997), (0.0679985, 0.1845, 0.114479))
link5:((-0.0679997, -0.068, -0.01), (0.0682466, 0.0990203, 0.267007))
link6:((-0.0661211, -0.0863635, -0.0707), (0.0661296, 0.0809, 0.0669138))
'''

# ['base_mount_joint', 'base_to_ptu_base', 'ptu_base', 'ptu_pan', 'ptu_tilt', 'ptu_mount', 'camera_joint', 'camera_joint_cheats']
def iiwaHeadLinks():
	links = 9*[None] + [Sh(Ba([(-0.1, -0.025, -0.05), (0.1, 0.025, 0.05)], name='kinect'))]
	return links

# ['base_foot_joint', 'base_joint', 'iiwa_joint_1', 'iiwa_joint_2', 'iiwa_joint_3', 'iiwa_joint_4', 'iiwa_joint_5', 'iiwa_joint_6', 'iiwa_joint_7', 'tool0_joint']
def iiwaArmLinks():
	links = [None] + [\
		Sh(Ba(((-0.136, -0.121384, -4.321e-07), (0.121243, 0.121384, 0.1575)), name='iiwa_link_0'), name='iiwa_link_0'),
		Sh(Ba(((-0.0854912, -0.115686, -0.01), (0.0854912, 0.0854874, 0.288)), name='iiwa_link_1'), name='iiwa_link_1'),
		Sh(Ba(((-0.0850566, -0.0854325, -0.0679994), (0.085929, 0.204501, 0.132107)), name='iiwa_link_2'), name='iiwa_link_2'),
		Sh(Ba(((-0.068, -0.068, -0.01), (0.0679997, 0.114593, 0.283494)), name='iiwa_link_3'), name='iiwa_link_3'),
		Sh(Ba(((-0.0679985, -0.0679989, -0.0679997), (0.0679985, 0.1845, 0.114479)), name='iiwa_link_4'), name='iiwa_link_4'),
		Sh(Ba(((-0.0679997, -0.068, -0.01), (0.0682466, 0.0990203, 0.267007)), name='iiwa_link_5'), name='iiwa_link_5'),
		Sh(Ba(((-0.0661211, -0.0863635, -0.0707), (0.0661296, 0.0809, 0.0669138)), name='iiwa_link_6'), name='iiwa_link_6')] + 2*[None]
	return links
