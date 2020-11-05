"""
	Author:    Prahar Bhatt
	Created:   3.12.2020

	Center for Advanced Manufacturing, University of Southern California.
"""

# Importing libraries
from OCC.Core.STEPControl import STEPControl_Reader
from OCC.Display.SimpleGui import *
from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_MakeFace,\
 BRepBuilderAPI_MakeEdge, BRepBuilderAPI_MakeWire
from OCC.Core.BRepAlgoAPI import BRepAlgoAPI_Section
from OCC.Core.gp import gp_Pln, gp_Dir, gp_Pnt, gp_Vec, gp_Ax2, gp_Dir
from OCC.Extend.ShapeFactory import get_boundingbox
from OCC.Extend.TopologyUtils import discretize_edge
from OCC.Core.BRepAdaptor import BRepAdaptor_Curve
from OCC.Core.GCPnts import GCPnts_AbscissaPoint
from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakeCylinder
import numpy
import time

# Class containing the basic functions
class Basic:

	# Defining variables
	time = 0

	# Fucntion to exit the GUI
	def Exit():
		quit()

	# Function to read step file
	def read_step(fileloc):
		step_reader = STEPControl_Reader()
		step_reader.ReadFile(fileloc)
		step_reader.TransferRoot()
		shape = step_reader.Shape()
		return shape

	# Function to check if a surface is clicked
	def recognize_clicked(shape, *kwargs):
		for s in shape:
			if s not in surfaces:
				surfaces.append(s)
				display.DisplayShape(s, color='WHITE', transparency= 0, update=True)
			else:
				surfaces.remove(s)
				display.DisplayShape(s, color='BLACK', update=True)

	# Function to create a menu in GUI
	def custom_menu():
		add_menu('File')
		add_function_to_menu('File', Basic.save_CSV)
		add_function_to_menu('File', Basic.Exit)
		add_menu('Build')
		add_function_to_menu('Build', Slicing.top_and_bottom_layers)
		add_function_to_menu('Build', Slicing.infill_conformal_layers)

	# Function to change the shape selection mode
	def shape_selection(type):
		if type=='F':
			display.SetSelectionModeFace()
		elif type=='V':
			display.SetSelectionModeVertex()
		elif type=='E':
			display.SetSelectionModeEdge()
		else:
			display.SetSelectionModeShape()

	# Function to display the path
	def display_path(lay, col, nozz_dia = 0):
		wire = BRepBuilderAPI_MakeWire()
		for i in range(1,len(lay)):
			if lay[i-1][0].Distance(lay[i][0]) < 8 and i < len(lay)-1:
				ray = BRepBuilderAPI_MakeEdge(lay[i-1][0],lay[i][0]).Edge()
				wire.Add(ray)
			else:
				display.DisplayShape(wire.Wire(), color = col, update = False)
				wire = BRepBuilderAPI_MakeWire()
			if i < len(lay)-1:
			 	display.DisplayShape(ray, color = col, update = False)
			else:
			 	display.DisplayShape(ray, color = col, update = True)

	# Function to display the normals
	def display_normals(lay):
		for la in lay:
			rayz = BRepBuilderAPI_MakeEdge(la[0], gp_Pnt((gp_Vec(la[0].XYZ()) + \
				la[3]*10).XYZ())).Edge()
			display.DisplayShape(rayz, color = 'BLUE1', update = False)

	#  Function to calculate the layer build time
	def display_time(lay):
		for i in range(1,len(lay)):
			distance  = lay[i-1][0].Distance(lay[i][0])
			if  distance < 8:
				Basic.time = Basic.time + distance/20
		print(Basic.time)

	# Function to display the desposited fibers
	def display_fiber(lay, col, nozz_dia):
		axis = gp_Ax2()
		for i in range(1,len(lay)):
				distance  = lay[i-1][0].Distance(lay[i][0])
				if  distance < 8:
					ray = gp_Dir(gp_Vec(lay[i-1][0],lay[i][0]))
					axis.SetLocation(lay[i-1][0])
					axis.SetDirection(ray)
					cylinder = BRepPrimAPI_MakeCylinder(axis, nozz_dia/2, distance)
					if i < len(lay) - 1:
						display.DisplayShape(cylinder.Shape(), color = col, update = False)
					else:
						display.DisplayShape(cylinder.Shape(), color = col, update = True)

	# Function to save the path to a csv file
	def save_CSV():
		if depth_of_layers < 0:
			layers.reverse()
		if not len(layers) == 0:		
			for i in range(0,len(layers)):
				text_file = open((part_name+" layer "+ str(i+1) +".csv"), "w")
				for j in range(0,len(layers[i])):
					text_file.write(str(round(layers[i][j][0].X(),3)))
					text_file.write(",")
					text_file.write(str(round(layers[i][j][0].Y(),3)))
					text_file.write(",")
					text_file.write(str(round(layers[i][j][0].Z(),3)))

					text_file.write(",")
					text_file.write(str(round(layers[i][j][1].X(),3)))
					text_file.write(",")
					text_file.write(str(round(layers[i][j][1].Y(),3)))
					text_file.write(",")
					text_file.write(str(round(layers[i][j][1].Z(),3)))

					text_file.write(",")
					text_file.write(str(round(layers[i][j][2].X(),3)))
					text_file.write(",")
					text_file.write(str(round(layers[i][j][2].Y(),3)))
					text_file.write(",")
					text_file.write(str(round(layers[i][j][2].Z(),3)))

					text_file.write(",")
					text_file.write(str(round(layers[i][j][3].X(),3)))
					text_file.write(",")
					text_file.write(str(round(layers[i][j][3].Y(),3)))
					text_file.write(",")
					text_file.write(str(round(layers[i][j][3].Z(),3)))

					text_file.write("\r\n")
				text_file.close()
			print (len(layers), "layers have been saved")
		if depth_of_layers < 0:
			layers.reverse()

# Class containing the functions for slicing
class Slicing:

	# Function to get apoint on curve
	def get_point_on_curve(bcurve, u):
		point = gp_Pnt()
		tangenty = gp_Vec()
		bcurve.D1(u, point, tangenty)
		vectorz=gp_Vec(0,0,1)
		vectorx = tangenty.Crossed(vectorz)
		vectorz = vectorx.Crossed(tangenty)
		vectorx.Normalize()
		tangenty.Normalize()
		vectorz.Normalize()
		return [point,vectorx,tangenty,vectorz]

	# Function to build the next layer
	def build_the_layer(path, displacement, opposite = False):
		new_path=[]
		for p in path:
			new_path.append([gp_Pnt((gp_Vec(p[0].XYZ()) + p[3]*displacement)\
				.XYZ()),p[1],p[2],p[3]])
		if opposite:
			new_path.reverse()
		return new_path

	# Function to get bounding box of the shape
	def get_boundingbox_dimensions(shape):
		xmin, ymin, zmin, xmax, ymax, zmax = get_boundingbox(shape)
		return xmax-xmin, ymax-ymin, zmax-zmin

	# Function to get the bounding box of multiple faces
	def get_surfaces_boundingbox(faces):
		minx = []
		miny = []
		minz = []
		maxx = []
		maxy = []
		maxz = []
		for face in faces:
			xmin, ymin, zmin, xmax, ymax, zmax = get_boundingbox(face)
			minx.append(xmin)
			miny.append(ymin)
			minz.append(zmin)	
			maxx.append(xmax)
			maxy.append(ymax)
			maxz.append(zmax)
		return min(minx), min(miny), min(minz), max(maxx), max(maxy), max(maxz)

	# Function to get the intersection between a plane and a shape
	def plane_shape_intersection(plane_face, shape):
		edges = []
		section = BRepAlgoAPI_Section(shape, plane_face)
		section.Approximation(True)
		section.Build()
		section_edges = section.SectionEdges()
		while (section_edges.IsEmpty()!=True):
			edges.append(section_edges.First())
			display.DisplayShape(section_edges.First(), color='BLACK', update=False)
			section_edges.RemoveFirst()
		return edges

	# Function to sort the surfaces
	def sort_surfaces(faces, along):
		min_along = []
		def get_min(e):
			return e['min']
		for i in range(0,len(faces)):
			xmin, ymin, xxx, xxx, xxx, xxx = get_boundingbox(faces[i])
			if along == 'X':
				min_along.append({'id':i, 'min':ymin})
			elif along == 'Y':
				min_along.append({'id':i, 'min':xmin})
		min_along.sort(key = get_min)
		new_faces = []
		for j in range(0, len(faces)):
			new_faces.append(faces[min_along[j]['id']])
		return new_faces			

	# Function to slice the surfaces
	def slice_selected_surfaces(nozz_dia, direc):
		slices = []
		counter1 = 0
		edge_clearance = nozz_dia/2
		xmin, ymin, zzz, xmax, ymax, zzz =\
		 Slicing.get_surfaces_boundingbox(surfaces)
		new_surfaces = Slicing.sort_surfaces(surfaces, direc)
		if direc=='X':
			imin=xmin
			imax=xmax
		elif direc=='Y':
			imin=ymin
			imax=ymax
		for i in numpy.arange(imin+edge_clearance, imax-edge_clearance+nozz_dia/2, \
			nozz_dia):
			if direc == 'X':
				plane = gp_Pln(gp_Pnt(i, 0., 0), gp_Dir(1., 0., 0.))
			elif direc =='Y':
				plane = gp_Pln(gp_Pnt(0., i, 0), gp_Dir(0., 1., 0.))
			face = BRepBuilderAPI_MakeFace(plane).Face()
			slices.append([])
			for surface in new_surfaces:
				slices[counter1].extend(Slicing.plane_shape_intersection(face,\
				 surface))
			counter1 = counter1 + 1
		return slices

	# Function to generate a conformal path over surface
	def generate_conformal_path(nozz_dia, slices, direc):
		layer=[]
		frames = []
		counter2 = 0
		edge_clearance = nozz_dia/2
		for s in slices:
			frames.append([])
			for j in range(0,len(s)):
				curve = BRepAdaptor_Curve(s[j])
				umin = curve.FirstParameter()
				umax = curve.LastParameter()
				if direc == 'X':
					umin_value = curve.Value(umin).Y()
					umax_value = curve.Value(umax).Y()
				elif direc == 'Y':
					umin_value = curve.Value(umin).X()
					umax_value = curve.Value(umax).X()
				if umin_value > umax_value:
					umax = curve.FirstParameter()
					umin = curve.LastParameter()
				length = GCPnts_AbscissaPoint().Length(curve)
				if j == 0:
					kmin = umin + (umax-umin)*edge_clearance/length
				else:
					kmin = umin
				if j == len(s)-1:
					kmax = umax - (umax-umin)*edge_clearance/length
				else:
					kmax = umax - (umax-umin)*min_point_dist/length
				length = GCPnts_AbscissaPoint().Length(curve,kmin,kmax)
				density = length/min_point_dist
				if density < 1:
					density = 1
				for k in numpy.arange(kmin, kmax, (kmax-kmin)/density):
					if k == kmax:
						break
					frames[counter2].append(Slicing.get_point_on_curve(curve,k))
				frames[counter2].append(Slicing.get_point_on_curve(curve,kmax))
			if counter2 % 2 != 0:
				frames[counter2].reverse()
			layer.extend(frames[counter2])
			counter2 = counter2 + 1
			# Basic.display_normals(layer)
		return layer

	# Function To generate top and bottom layers
	def top_and_bottom_layers():
		small_slices = Slicing.slice_selected_surfaces(small_nozz_dia, direction)
		small_layer_template = Slicing.generate_conformal_path(small_nozz_dia,\
		 small_slices, direction)
		layers.insert(0, Slicing.build_the_layer(small_layer_template, 0, False))
		if simulate == False:
			Basic.display_path(layers[0], 'BLACK')
		else:
			Basic.display_fiber(layers[0], 'BLACK', small_nozz_dia)
		Basic.display_time(layers[0])
		layers.append(Slicing.build_the_layer(small_layer_template, \
			depth_of_layers, True))
		if simulate == False:
			Basic.display_path(layers[-1], 'BLACK')
		else:
			Basic.display_fiber(layers[-1], 'BLACK', small_nozz_dia)
		Basic.display_time(layers[-1])
		
	# Function to generate infill conformal layers
	def infill_conformal_layers():
		if direction == 'X':
			alternate_direction = 'Y'
		else:
			alternate_direction = 'X'
		alternate_direction = direction
		big_slices = Slicing.slice_selected_surfaces(big_nozz_dia,\
		 alternate_direction)
		big_layer_template = Slicing.generate_conformal_path(big_nozz_dia,\
		 big_slices, alternate_direction)
		sign = depth_of_layers/ abs(depth_of_layers)
		start = ((small_nozz_dia/2) + (big_nozz_dia/2)) * sign
		end = depth_of_layers - start
		step = sign * big_nozz_dia
		color_id = 0
		for i in numpy.arange(start, end, step):
			layers.append(Slicing.build_the_layer(big_layer_template, i, False))
			if simulate == False:
				Basic.display_path(layers[-1], 'GREEN')
			else:
				Basic.display_fiber(layers[-1], colour[color_id%5], big_nozz_dia)
			Basic.display_time(layers[-1])
			color_id = color_id + 1
		layers.append(Slicing.build_the_layer(big_layer_template, end, False))
		if simulate == False:
			Basic.display_path(layers[-1], 'GREEN')
		else:
			Basic.display_fiber(layers[-1], colour[color_id%5], big_nozz_dia)
		Basic.display_time(layers[-1])

# Execution begins here
if __name__ == "__main__":
	display, start_display, add_menu, add_function_to_menu = init_display()

	# Defining global variables
	global part_shape, big_nozz_dia, surfaces, direction,\
	 min_point_dist, layers, depth_of_layers, simulate, colour
	
	# Initialize the global variables
	colour =['RED','GREEN','YELLOW','BLUE1','WHITE']
	colour = ['YELLOW','YELLOW','YELLOW','YELLOW','YELLOW']
	surfaces = []
	big_nozz_dia = 1.2
	small_nozz_dia = 0.6
	direction = 'Y'
	min_point_dist = 2
	layers = []
	depth_of_layers = 6.6#-3.0#6.6#
	simulate = True
	part_name='mrp2'
	part_shape = Basic.read_step(('CAD/'+part_name+'.stp'))
	
	# Display in GUI
	display.DisplayShape(part_shape, color='BLACK',transparency=0.5, update=True)
	print(Slicing.get_boundingbox_dimensions(part_shape))
	Basic.shape_selection('F')
	display.register_select_callback(Basic.recognize_clicked)
	Basic.custom_menu()

	# Begin GUI
	start_display()