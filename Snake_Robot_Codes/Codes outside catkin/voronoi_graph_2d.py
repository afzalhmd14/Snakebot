# Author: Chaitanya Pb

# This code creates a random obstacle map. It uses the Voronoi Graph Method to automatically 
# generate a path through the created obstacle field and displays the resultant path.

# Standalone: It does not call functions from any other file.

# Doesn't contain paths to external files.

import PIL as pil
import numpy as np
import random as rnd
import scipy.spatial as scsptl
import scipy.sparse as scsprs
import scipy.interpolate as scintr
from skimage import filter, io
import matplotlib.pyplot as plt

# Function generates a random obstacle map
def generate_obstacle_map(start_pnt, end_pnt, k):
    
    obstacles = []
    obst_image = np.ones([k*end_pnt[0], k*end_pnt[1]])
    
    obst_num = rnd.randint(3, c_dict['max_obst'])

    for obst_id in range(1, obst_num+1):
        obst_type = rnd.randint(1, len(c_dict['name_types']))
        obst_xy = (rnd.uniform(c_dict['x_min'], c_dict['x_max']), rnd.uniform(c_dict['y_min'], c_dict['y_max']))
        obst_image_xy = tuple(int(round(k*element)) for element in obst_xy)
        
        if c_dict['name_types'][obst_type-1] == 'cylinder':    
            obst_dim = [rnd.uniform(c_dict['r_min'], c_dict['r_max'])]
            obst_image_dim = tuple(int(round(k*element)) for element in obst_dim)
            for x in range(-obst_image_dim[0], obst_image_dim[0]):
                for y in range(-obst_image_dim[0], obst_image_dim[0]):
                    if x*x + y*y <= obst_image_dim[0]*obst_image_dim[0]:
                        obst_image[obst_image_xy[0]+x, obst_image_xy[1]+y] = 0
                    
        elif c_dict['name_types'][obst_type-1] == 'cuboid':
            obst_len = rnd.uniform(c_dict['l_min'], c_dict['l_max'])
            obst_dim = [obst_len, c_dict['peri_const']-obst_len]#, rnd.uniform(c_dict['a_min'], c_dict['a_max'])]
            obst_image_dim = tuple(int(round(k*element)) for element in obst_dim)
            for x in range(int(-obst_image_dim[0]/2), int(obst_image_dim[0]/2)):
                for y in range(int(-obst_image_dim[1]/2), int(obst_image_dim[1]/2)):
                    obst_image[obst_image_xy[0]+x, obst_image_xy[1]+y] = 0
                
        obstacles.append([obst_id, c_dict['name_types'][obst_type-1], obst_xy, obst_dim])
        
    return obst_image, obstacles

# Function generates Voronoi data
def generate_voronoi_data(obst_img, coords, k):
    
    edge_img = filter.sobel(obst_img)
    for x in range(0, int(edge_img.shape[0])):
        for y in range(0, int(edge_img.shape[1])):
            if edge_img[(x,y)] != 0:
                coords.append((float(x)/k, float(y)/k))
    
    sep_coords = list(zip(*coords)) 
    arr_coords = np.asarray(sep_coords).T

    return edge_img, sep_coords, arr_coords

# Function converts Voronoi data to graph
def create_voronoi_graph(arr_coords, start_pnt, end_pnt):
    
    vor = scsptl.Voronoi(arr_coords)

    arr_vertices = vor.vertices
    arr_startend = np.asarray([start_pnt, end_pnt])
    distances = scsptl.distance.cdist(arr_startend, arr_vertices)
    start_node = np.argmin(distances[0])
    end_node = np.argmin(distances[1])

    graph = np.zeros([len(vor.vertices), len(vor.vertices)])
    for edgep in vor.ridge_points:
        edgev = vor.ridge_dict[tuple(edgep)]
        if edgev[0] != -1 and edgev[1] != -1:
            edgev_wgt = np.linalg.norm(vor.vertices[edgev[0]] - vor.vertices[edgev[1]])
            edgep_wgt = np.linalg.norm(vor.points[edgep[0]] - vor.points[edgep[1]])/2
            graph[edgev[0]][edgev[1]] = max(edgev_wgt - 2*edgep_wgt, 0.01)
            graph[edgev[1]][edgev[0]] = graph[edgev[0]][edgev[1]]
    
    return vor, graph, start_node, end_node

# Function finds best path from graph and fits spline
def find_best_path_and_spline(vor, graph, start_node, end_node):
    
    sp_matrix, pred = scsprs.csgraph.shortest_path(graph, return_predecessors=True)

    pred = pred[start_node]; curr = end_node
    sp_indices = [end_node]
    if start_node != end_node:
        while pred[curr] != start_node:
            sp_indices.insert(0, pred[curr])
            curr = pred[curr]
        sp_indices.insert(0, start_node)
    sp_coords = vor.vertices[sp_indices]
    sp_coords = np.vstack([np.asarray(start_pnt), sp_coords, np.asarray(end_pnt)])
    sp_xcoords, sp_ycoords = list(zip(*sp_coords))
    
    spline_tck, spline_u = scintr.splprep([sp_xcoords, sp_ycoords], s=0.5)
    spline_npnt = scintr.splev(np.linspace(0,1,num=1000), spline_tck)
    
    return sp_xcoords, sp_ycoords, spline_npnt

# Function generates path by Voronoi Graph Method
def generate_voronoi_path(obst_img, coords, start_pnt, end_pnt, k):
    edge_img, sep_coords, arr_coords = generate_voronoi_data(obst_img, coords, k)
    vor, graph, start_node, end_node = create_voronoi_graph(arr_coords, start_pnt, end_pnt)
    sp_xcoords, sp_ycoords, spline_npnt = find_best_path_and_spline(vor, graph, start_node, end_node)

    plt.scatter(sep_coords[0], sep_coords[1])
    plt.plot(sp_xcoords, sp_ycoords, 'ro')
    plt.plot(spline_npnt[0], spline_npnt[1], 'r-')
    plt.show()

    return

# Define necessary variables
c_dict = {'max_obst' : 10,
              'name_types' : ['cylinder', 'cuboid'],
              'x_min' : 1,
              'x_max' : 14,
              'y_min' : 1,
              'y_max' : 14,
              'r_min' : 0.1,
              'r_max' : 0.9,
              'l_min' : 0.1,
              'l_max' : 1.9,
              'a_min' : 0,
              'a_max' : 180,
              'peri_const' : 2}

k = 10
start_pnt = (0,0)
end_pnt = (15,15)
coords = [start_pnt, end_pnt]

# Generate random obstacles and automated path
obst_img, obstacles = generate_obstacle_map(start_pnt, end_pnt, k)
generate_voronoi_path(obst_img, coords, start_pnt, end_pnt, k)
