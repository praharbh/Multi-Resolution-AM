"""
	Author:    Prahar Bhatt
	Created:   4.17.2020

	Center for Advanced Manufacturing, University of Southern California.
"""

# Importing libraries
import ikpy
from OCC.Core.gp import gp_Pnt, gp_Vec, gp_Trsf, gp_Dir, gp_Ax1,\
	gp_Mat, gp_Quaternion, gp_GTrsf, gp_XYZ
from OCC.Core.STEPControl import STEPControl_Reader
from OCC.Display.SimpleGui import *
from OCC.Core.TopLoc import TopLoc_Location
from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_MakeEdge,\
 BRepBuilderAPI_MakeWire
import numpy
import math
import pandas as pd
import time

# Class for containing basic functions
class Basic():	

	# Function to read csv file
	def read_points(csv_name):
		readCSV = pd.read_csv(csv_name,sep=',',header=None,dtype=float)
		values = readCSV.values
		points=[]
		for j in range(0,len(values)):
			points.append([gp_Pnt(values[j][0]+700,values[j][1],values[j][2]+600),\
			gp_Vec(values[j][3],values[j][4],values[j][5]),\
			gp_Vec(values[j][6],values[j][7],values[j][8]),\
			gp_Vec(values[j][9],values[j][10],values[j][11])])
		return points

	# Function to display the path
	def display_path(points, colour='RED'):
		for i in range(1,len(points)):
			if points[i-1][0].Distance(points[i][0]) < 10:
				ray = BRepBuilderAPI_MakeEdge(points[i-1][0],points[i][0]).Edge()
				display.DisplayShape(ray, color = colour, update = False)

# Class containing the robot kinematic functions
class Kinematics():

	# Function to create a kinematic chain from URDF file
	def read_URDF(file_name):
		return ikpy.chain.Chain.from_urdf_file(file_name)
	
	# Function to calculate the forward kinematics
	def FK(robot_model, joint_var):
		mod_joint_var = numpy.concatenate([[0],joint_var])
		# mod_joint_var.append(0)
		A = robot_model.forward_kinematics(mod_joint_var,True)
		idx = len(A) - 1
		P = gp_Pnt(A[idx][0][3]*1000,A[idx][1][3]*1000,A[idx][2][3]*1000)
		Vx = gp_Vec(A[idx][0][0]*1000,A[idx][1][0]*1000,A[idx][2][0]*1000)
		Vy = gp_Vec(A[idx][0][1]*1000,A[idx][1][1]*1000,A[idx][2][1]*1000)
		Vz = gp_Vec(A[idx][0][2]*1000,A[idx][1][2]*1000,A[idx][2][2]*1000)
		Vx.Normalize()
		Vy.Normalize()
		Vz.Normalize()
		return [P, Vx, Vy, Vz]

	# Function to calculate the inverse kinematics
	def IK(robot_model, frame, seed = []):
		if len(seed) != 0:
			target_seed = numpy.concatenate([[0],seed])
		else:
			target_seed = None
		target = numpy.eye(4)
		target[:3, 3] = [frame[0].X()/1000, frame[0].Y()/1000,\
		 frame[0].Z()/1000]
		target[:3, 0] = [frame[1].X(), frame[1].Y(), frame[1].Z()]
		target[:3, 1] = [frame[2].X(), frame[2].Y(), frame[2].Z()]
		target[:3, 2] = [frame[3].X(), frame[3].Y(), frame[3].Z()]

		joint_var = robot_model.inverse_kinematics(target, target_seed)
		joint_var = numpy.delete(joint_var,0)

		new_frame = Kinematics.FK(robot_model, joint_var)
		
		distance = frame[0].Distance(new_frame[0])
		alpha = frame[1].Angle(new_frame[1])*180/math.pi
		beta = frame[2].Angle(new_frame[2])*180/math.pi
		gamma = frame[3].Angle(new_frame[3])*180/math.pi
		if distance > 0.1 or alpha > 1 or beta > 1 or gamma > 1:
			print("High Error: [" + str(distance) + "," + str(alpha) + "," +\
			 str(beta) + "," + str(gamma) + "]")
			return numpy.zeros((len(robot_model.links)-1))
		else:
			return joint_var

	# Function to apply the the tool transform
	def reverse_tool_transform(points):
		M = gp_Mat(-0.955248, -0.295806, 0.0000000000,\
		-0.295806, 0.955248, 0.0000000000,\
		0.0000000000, 0.0000000000, -1.00000)
		P = gp_Pnt(-20.5609, -11.9004, 251.300)
		tool_trsf = gp_Trsf()
		tool_trsf.SetTransformation(gp_Quaternion(M),\
		gp_Vec(gp_Pnt(),P))
		tool_trsf.Invert()
		
		M = gp_Mat(0,0,1,0,-1,0,1,0,0)
		P = gp_Pnt(100,0,0)
		ee_trsf = gp_Trsf()
		ee_trsf.SetTransformation(gp_Quaternion(M),\
		gp_Vec(gp_Pnt(),P))
		ee_trsf.Invert()
		
		total_trsf = tool_trsf.Multiplied(ee_trsf)
		new_points = []
		for point in points:
			new_point=[]
			M = gp_Mat(point[1].X(),point[2].X(),point[3].X(),\
				point[1].Y(),point[2].Y(),point[3].Y(),
				point[1].Z(),point[2].Z(),point[3].Z())
			P = point[0]
			point_trsf = gp_Trsf()
			point_trsf.SetTransformation(gp_Quaternion(M),\
			gp_Vec(gp_Pnt(),P))
			new_point_trsf = point_trsf.Multiplied(total_trsf)
			M = new_point_trsf.VectorialPart()
			new_point.append(gp_Pnt(new_point_trsf.TranslationPart()))
			new_point.append(gp_Vec(M.Column(1)).Normalized())
			new_point.append(gp_Vec(M.Column(2)).Normalized())
			new_point.append(gp_Vec(M.Column(3)).Normalized())
			new_points.append(new_point)
		return new_points

# Class robot conatining the plotting functions
class Plot():

	# Function to read step files
	def read_step(file_loc):
		step_reader = STEPControl_Reader()
		step_reader.ReadFile(file_loc)
		step_reader.TransferRoot()
		shape = step_reader.Shape()
		return shape
	
	# Function to update the link locations
	def update(links, T, tool = None):
		for i in range(0,len(links)):
			display.Context.SetLocation(links[i], TopLoc_Location(T[i]))
			if tool != None and i == len(links)-1:
				M = gp_Mat(0,0,1,0,-1,0,1,0,0)
				P = gp_Pnt(100,0,0)
				T_ee = gp_Trsf()
				T_ee.SetTransformation(gp_Quaternion(M),\
				gp_Vec(gp_Pnt(),P))
				display.Context.SetLocation(tool, TopLoc_Location(T[i].Multiplied(T_ee)))
		display.Context.UpdateCurrentViewer()

	# Function to create a custom menu
	def custom_menu():
		add_menu('File')
		add_function_to_menu('File', Plot.Exit)

	# Function to exit GUI
	def Exit():
		quit()

	# Function to calculte the FK for plotting
	def FK(robot_model, joint_var):
		mod_joint_var = numpy.concatenate([[0],joint_var])
		A = robot_model.forward_kinematics(mod_joint_var,True)
		# A = A[:-1]
		T = []
		for a in A:
			M = gp_Mat(a[0][0],a[0][1],a[0][2],\
				a[1][0],a[1][1],a[1][2],\
				a[2][0],a[2][1],a[2][2])
			P = gp_Pnt(a[0][3]*1000,a[1][3]*1000,a[2][3]*1000)
			trsf = gp_Trsf()
			trsf.SetTransformation(gp_Quaternion(M),\
					gp_Vec(gp_Pnt(),P))
			T.append(trsf)
		return T

	# Function tp initilize the robot model
	def Initialize(robot_model):
		T = Plot.FK(robot_model,numpy.zeros((len(robot_model.links)-1)))
		shapes=[]
		for i in range(0,len(robot_model.links)):
			shape = Plot.read_step(robot_name+'/'+ str(i) + '.stp')
			IT = TopLoc_Location(T[i]).Inverted()
			shape.Move(IT)
			shapes.append(display.DisplayShape(shape, color='BLUE1',\
			transparency= 0, update=True))
		return shapes

	# Function to dispay the coordinates on the path
	def display_coord(frames):
		for frame in frames:
			[pnt,vecx,vecy,vecz] = frame
			rayx = BRepBuilderAPI_MakeEdge(pnt, gp_Pnt((gp_Vec(pnt.XYZ()) +\
			vecx*30).XYZ())).Edge()
			rayy = BRepBuilderAPI_MakeEdge(pnt, gp_Pnt((gp_Vec(pnt.XYZ()) +\
		 	vecy*30).XYZ())).Edge()
			rayz = BRepBuilderAPI_MakeEdge(pnt, gp_Pnt((gp_Vec(pnt.XYZ()) +\
		 	vecz*30).XYZ())).Edge()
			display.DisplayShape(rayx, color = 'RED', update = False)
			display.DisplayShape(rayy, color = 'GREEN', update = False)
			display.DisplayShape(rayz, color = 'BLUE1', update = False)

	# Function to display the path
	def display_path(lay, col):
		wire = BRepBuilderAPI_MakeWire()
		for i in range(1,len(lay)):
			if lay[i-1][0].Distance(lay[i][0]) < 50:
				ray = BRepBuilderAPI_MakeEdge(lay[i-1][0],lay[i][0]).Edge()
				wire.Add(ray)
		display.DisplayShape(wire.Wire(), color = col, update = False)

# Execution begins here
if __name__ == "__main__":
	display, start_display, add_menu, add_function_to_menu = init_display()
	
	# Create custom menu
	Plot.custom_menu()
	
	# Initialize variables
	robot_name = 'GP12'
	robot_model = Kinematics.read_URDF(robot_name+'/URDF.xml')
	tool_name = 'nozzle'
	tool_stp = Plot.read_step('CAD/'+tool_name+'.stp')
	tool_shape = display.DisplayShape(tool_stp, color='Black',\
	 		transparency= 0, update=True)
	part_name = 'mrp1'
	part_shape = Plot.read_step('CAD/'+part_name+'.stp')
	robot_shapes = Plot.Initialize(robot_model)

	# Read path
	global frames
	frames = Basic.read_points("CSV/mrp1.csv")
	Plot.display_path(frames,'RED')
	new_frames = Kinematics.reverse_tool_transform(frames)
	# Plot.display_coord(new_frames)
	zero_joint = numpy.zeros((len(robot_model.links)-1))

	# Simulate the robot
	Plot.update(robot_shapes, Plot.FK(robot_model,\
	 zero_joint),tool_shape)
	prev_joint = zero_joint
	new_joints = []
	for f in range(0,len(frames)):
		new_joint = Kinematics.IK(robot_model,new_frames[f],zero_joint)
		if new_joint.all() == 0:
			print ('skip')
		else:
			new_joints.append(new_joint)
			prev_joint = new_joint
			Plot.update(robot_shapes, Plot.FK(robot_model, new_joint), tool_shape)
		break

	# Begin the GUI
	start_display()
