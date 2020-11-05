"""
	Author:    Prahar Bhatt
	Created:   4.22.2020

	Center for Advanced Manufacturing, University of Southern California.
"""

# Importing libraries
from OCC.Core.STEPControl import STEPControl_Reader
from OCC.Display.SimpleGui import *
from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_MakeFace,\
 BRepBuilderAPI_MakeEdge, BRepBuilderAPI_MakeWire
from OCC.Core.BRepAlgoAPI import BRepAlgoAPI_Section
from OCC.Core.gp import gp_Pln, gp_Dir, gp_Pnt, gp_Vec, gp_Ax2, gp_Dir, gp_XOY
from OCC.Extend.ShapeFactory import get_boundingbox
from OCC.Extend.TopologyUtils import *
from OCC.Core.BRepAdaptor import BRepAdaptor_Curve
from OCC.Core.GCPnts import GCPnts_AbscissaPoint
from OCC.Core.BRepAlgoAPI import BRepAlgoAPI_Cut
from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakeCylinder
from OCC.Core.Geom import Geom_Circle
from OCC.Core.BRepOffsetAPI import BRepOffsetAPI_MakePipe
import numpy
import time

# Class contaning basic functions
class Basic:

	# Defining variables
	time = 0

	# Function to exit the GUI
	def Exit():
		quit()

	# Function to read a step file
	def read_step(fileloc):
		step_reader = STEPControl_Reader()
		step_reader.ReadFile(fileloc)
		step_reader.TransferRoot()
		shape = step_reader.Shape()
		return shape

	# Function to register a clicked surface
	def recognize_clicked(shape, *kwargs):
		for s in shape:
			if s not in surfaces:
				surfaces.append(s)
				display.DisplayShape(s, color='White', transparency= 0.5, update=True)
			else:
				surfaces.remove(s)
				display.DisplayShape(s, color='BLACK', transparency= 0.5,update=True)

	# Function to create the menu
	def custom_menu():
		add_menu('File')
		add_function_to_menu('File', Basic.save_CSV)
		add_function_to_menu('File', Basic.Exit)
		add_menu('Build')
		add_function_to_menu('Build', Slicing.generate_surface_layer)
		add_function_to_menu('Build', Slicing.generate_middle_layers)
	
	# Function to toggle the selection type
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
			ray = BRepBuilderAPI_MakeEdge(lay[i-1][0],lay[i][0]).Edge()
			if i==len(lay)-1:
			 	display.DisplayShape(ray, color = col, update = True)
			else:
			 	display.DisplayShape(ray, color = col, update = False)

	# Function to display normals on the path
	def display_normals(lay):
		for la in lay:
			rayz = BRepBuilderAPI_MakeEdge(la[0], gp_Pnt((gp_Vec(la[0].XYZ())\
			 + la[3]*10).XYZ())).Edge()
			display.DisplayShape(rayz, color = 'BLUE1', update = False)

	# Function to display the depsoited fiber
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

	# Function to calculate the layer build time
	def display_time(lay):
		for i in range(1,len(lay)):
			distance  = lay[i-1][0].Distance(lay[i][0])
			if  distance < 8:
				Basic.time = Basic.time + distance/20
		print(Basic.time)

	# # Obsolete function to display a pipe along the path
	# def display_pipe(lay, col, nozz_dia):
	# 	circle = Geom_Circle(gp_XOY(), nozz_dia/2)
	# 	circle_edge = BRepBuilderAPI_MakeEdge(circle).Edge()
	# 	circle_wire = BRepBuilderAPI_MakeWire()
	# 	circle_wire.Add(circle_edge)
	# 	circle_face = BRepBuilderAPI_MakeFace(circle_wire.Wire()).Face()
	# 	wire = BRepBuilderAPI_MakeWire()
	# 	for i in range(1,len(lay)):
	# 		if lay[i-1][0].Distance(lay[i][0]) < 5 and i < len(lay)-1:
	# 			ray = BRepBuilderAPI_MakeEdge(lay[i-1][0],lay[i][0]).Edge()
	# 			wire.Add(ray)
	# 		else:
	# 			pipe = BRepOffsetAPI_MakePipe(wire.Wire(), circle_face).Shape()
	# 			display.DisplayShape(pipe, color = col, update = True)
	# 			wire = BRepBuilderAPI_MakeWire()

	# Function to save the path as a csv file
	def save_CSV():
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

					text_file.write("\n")
				text_file.close()
			print (len(layers), "layers have been saved")

# Class containing slicing functions
class Slicing:

	# Defining variables
	tolerance = 1e-1

	# Function to get a point on a curve
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

	# Function to get a bounding box for the surface
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

	# Function to get the intersection betaeen a plane and a shape
	def plane_shape_intersection(plane_face, shape):
		edges = []
		section = BRepAlgoAPI_Section(shape, plane_face)
		section.Approximation(True)
		section.Build()
		section_edges = section.SectionEdges()
		while (section_edges.IsEmpty()!=True):
			edges.append(section_edges.First())
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

	# Function generate a path over the surfaces
	def generate_layer_path(faces, nozz_dia, direc, sur_nozz_dia = 0):
		slices = []
		counter1 = 0
		layer=[]
		frames = []
		counter2 = 0
		edge_clearance = (nozz_dia+sur_nozz_dia)/2	
		xmin, ymin, zzz, xmax, ymax, zzz =\
		 Slicing.get_surfaces_boundingbox(faces)
		new_surfaces = Slicing.sort_surfaces(faces, direc)
		if direc == 'X':
			imin=xmin
			imax=xmax
		elif direc=='Y':
			imin=ymin
			imax=ymax
		for i in numpy.arange(imin+edge_clearance, imax-edge_clearance, nozz_dia):
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
		return layer

	# Function to get the end points of a edge
	def get_edge_endpoints(edge):
		end_points = []
		curve = BRepAdaptor_Curve(edge)
		end_points.append(curve.Value(curve.FirstParameter()))
		end_points.append(curve.Value(curve.LastParameter()))
		return end_points

	# Function to check if two edges connect
	def do_edges_connect(edgeA,edgeB):
		distance = []
		edgeA_end_points = Slicing.get_edge_endpoints(edgeA)
		edgeB_end_points = Slicing.get_edge_endpoints(edgeB)
		distance.append(edgeA_end_points[0].Distance(edgeB_end_points[0]))
		distance.append(edgeA_end_points[0].Distance(edgeB_end_points[1]))
		distance.append(edgeA_end_points[1].Distance(edgeB_end_points[0]))
		distance.append(edgeA_end_points[1].Distance(edgeB_end_points[1]))
		if min(distance) < Slicing.tolerance:
			return True
		else:
			return False

	# Function to check if two edges loop
	def do_edges_loop(edgeA,edgeB):
		distance = []
		edgeA_end_points = Slicing.get_edge_endpoints(edgeA)
		edgeB_end_points = Slicing.get_edge_endpoints(edgeB)
		distance.append(edgeA_end_points[0].Distance(edgeB_end_points[0]))
		distance.append(edgeA_end_points[0].Distance(edgeB_end_points[1]))
		distance.append(edgeA_end_points[1].Distance(edgeB_end_points[0]))
		distance.append(edgeA_end_points[1].Distance(edgeB_end_points[1]))
		num = 0
		if distance[0] < Slicing.tolerance:
			num = num +1 
		if distance[1] < Slicing.tolerance:
			num = num +1
		if distance[2] < Slicing.tolerance:
			num = num +1
		if distance[3] < Slicing.tolerance:
			num = num +1
		if num == 2:
			return True
		else:
			return False

	# Function to remove one face from the other
	def cut_face_from_face(bigface,smallface):
		cut = BRepAlgoAPI_Cut(bigface,smallface)
		it = TopoDS_Iterator(cut.Shape())
		return it.Value()

	# Function to generate faces from wires
	def get_layer_faces(wires):
		faces = []
		bounds = []
		area = []
		for wire in wires:
			face = BRepBuilderAPI_MakeFace(wire).Face()
			faces.append(face)
			xmin, ymin, zzz, xmax, ymax, zzz = get_boundingbox(face)
			area.append((xmax - xmin) * (ymax - ymin)) 
			bounds.append([ xmin, ymin, xmax, ymax])
		return faces
		for i in range (0,len(faces)):
			for j in range(0,len(faces)):
				if i == j:
					continue
				if (bounds[i][0] < bounds[j][0]) and (bounds[i][1] < bounds[j][1]) and\
				(bounds[i][2] > bounds[j][2]) and (bounds[i][3] > bounds[j][3]):
					faces[i] = Slicing.cut_face_from_face(faces[i],faces[j])
					faces.remove(faces[j])
		return faces

	# Function to generate planer slices
	def generate_planar_slices(nozz_dia, sur_nozz_dia):
		xmin, ymin, zmin, xmax, ymax, zmax = get_boundingbox(part_shape)
		print(xmax-xmin, ",", ymax-ymin, ",", zmax-zmin)
		wires = []
		slices = []
		contours = []
		for z in numpy.arange(zmin+(nozz_dia/2)+(sur_nozz_dia/2), \
			zmax-(nozz_dia/2)-(sur_nozz_dia/2), nozz_dia):
			plane = gp_Pln(gp_Pnt(0., 0., z), gp_Dir(0., 0., 1.))
			slices.append(Slicing.plane_shape_intersection(plane, part_shape))
		for s in range(0,len(slices)):
			wire = []
			wires.append([])
			while len(slices[s]) != 0:
				for i in range(0,len(slices[s])):
					if len(wire) == 0:
						wire.append(slices[s][i])
						slices[s].remove(slices[s][i])
						break
					elif Slicing.do_edges_connect(slices[s][i],wire[-1]):
						wire.append(slices[s][i])
						slices[s].remove(slices[s][i])
						break
				if Slicing.do_edges_connect(wire[0],wire[-1]):
					if len(wire)>2:
						wires[s].append(wire)
						wire = []
					elif len(wire) == 2 and Slicing.do_edges_loop(wire[0],wire[-1]):
						wires[s].append(wire)
						wire = []
		for k in range(0,len(wires)):
			contours.append([])
			for l in range(0,len(wires[k])):
				make_wire = BRepBuilderAPI_MakeWire()
				for edge in wires[k][l]:
					make_wire.Add(edge)
				try:
					made_wire = make_wire.Wire()
					contours[k].append(made_wire)
				except:
					print("Skipped a contour!")
					continue
		contour_faces = []
		for contour in contours:
			if len(contour) == 1:
				contour_faces.append(BRepBuilderAPI_MakeFace(contour[0]).Face())
			else:
				contour_faces.extend(Slicing.get_layer_faces(contour))
		# for l in range(0,len(contour_faces)):
		#   display.DisplayShape(contour_faces[l], color=colour[l%5],\
		#   	transparency=0.95, update=True)
		return contour_faces

	# Function to generate the conformal surface layer 
	def generate_surface_layer():
		layers.append(Slicing.generate_layer_path(surfaces, small_nozz_dia,\
		 direction))
		Basic.display_path(layers[-1], 'BLACK', small_nozz_dia)
		Basic.display_time(layers[-1])
		surfaces.clear()

	# Function to generate the middle planar layers
	def generate_middle_layers():
		if direction == 'X':
			alternate_direction = 'Y'
		else:
			alternate_direction = 'X'
		alternate_direction = direction
		planar_faces = Slicing.generate_planar_slices(big_nozz_dia, small_nozz_dia)
		for i in range(0,len(planar_faces)):
			layers.append(Slicing.generate_layer_path([planar_faces[i]],\
			 big_nozz_dia, alternate_direction, small_nozz_dia))
			Basic.display_fiber(layers[-1], colour[i%5], big_nozz_dia)
			Basic.display_time(layers[-1])

# Execution begins here
if __name__ == "__main__":

	# Initilizing the GUI
	display, start_display, add_menu, add_function_to_menu = init_display()

	# Defining global variables
	global part_shape, big_nozz_dia, surfaces, direction, colour,\
	 min_point_dist, layers, all_surfaces 
	
	# Initializing the global variables
	colour =['RED','GREEN','YELLOW','BLUE1','WHITE']
	colour = ['YELLOW','YELLOW','YELLOW','YELLOW','YELLOW']
	surfaces = []
	all_surfaces = []
	big_nozz_dia = 1.2
	small_nozz_dia = 8
	direction = 'Y'
	min_point_dist = 20
	layers = []
	part_name='bighood'
	part_shape = Basic.read_step(('CAD/'+part_name+'.stp'))
	display.DisplayShape(part_shape, color='BLACK',transparency=0.5, update=True)

	# Begining the GUI
	Basic.shape_selection('F')
	display.register_select_callback(Basic.recognize_clicked)
	Basic.custom_menu()
	start_display()