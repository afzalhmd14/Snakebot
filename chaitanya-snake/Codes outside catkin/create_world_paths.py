# Author: Chaitanya Pb

# This code creates a random obstacle map. It generates automated paths (via Voronoi Graph Method
# and Distance Transform Method) through the created obstacle field and saves these paths to files.
# It also creates two Gazebo world files (with obstacles and markers) in XML which are useful for 
# simulating the newly created environment.

# Uses these files to run: vg_path, dt_path, vg_random_obstacle.world and dt_random_obstacle.world

# Contains paths to external files.

import PIL as pil
import numpy as np
import random as rnd
import scipy.spatial as scsptl
import scipy.sparse as scsprs
import scipy.ndimage as scndim
import scipy.interpolate as scintr
from skimage import filter, io
import matplotlib.pyplot as plt

# Function writes the initial code for world file
def code_initial():
    
    init_def = "<?xml version='1.0' ?>\n<sdf version='1.4'>\n"
    grnd_def = "\n\t\t<include>\n\t\t\t<uri>model://ground_plane</uri>\n\t\t</include>\n"
    sunn_def = "\n\t\t<include>\n\t\t\t<uri>model://sun</uri>\n\t\t</include>\n"
    phys_def = "\n\t\t<physics type = 'ode'>\n\t\t\t<real_time_update_rate>0.0</real_time_update_rate>\n\t\t</physics>\n"
    wrld_def = "\n\t<world name='default'>\n" + phys_def + grnd_def + sunn_def

    worldfile.write(init_def + wrld_def)

# Function writes ending code for world file
def code_ending():

    endg_def = "\n\t</world>\n</sdf>"

    worldfile.write(endg_def)

# Function writes code for an obstacle in world file
def code_obstacle(obst_type, obst_loc, obst_dim):

    global cyl_cnt, cub_cnt

    if obst_type == 'cylinder':
        cyl_cnt += 1
        model_name = 'cylinder_' + str(cyl_cnt)
        dimn_def = "\t\t\t\t\t\t\t<radius>" + str(obst_dim[0]) + "</radius>\n\t\t\t\t\t\t\t<length>1</length>\n"
        geom_def = "\t\t\t\t\t<geometry>\n\t\t\t\t\t\t<cylinder>\n" + dimn_def + "\t\t\t\t\t\t</cylinder>\n\t\t\t\t\t</geometry>\n"
    elif obst_type == 'cuboid':
        cub_cnt += 1
        model_name = 'cuboid_' + str(cub_cnt)
        dimn_def = "\t\t\t\t\t\t\t<size>" + str(obst_dim[0]) + " " + str(obst_dim[1]) + " 1</size>\n"
        geom_def = "\t\t\t\t\t<geometry>\n\t\t\t\t\t\t<box>\n" + dimn_def + "\t\t\t\t\t\t</box>\n\t\t\t\t\t</geometry>\n"

    stat_def = "\t\t\t<static>1</static>\n"
    pose_def = "\t\t\t<pose>" + str(-obst_loc[0]) + " " + str(-obst_loc[1]) + " 0.5 0 0 0</pose>\n"
    coll_def = "\t\t\t\t<collision name='collision'>\n" + geom_def + "\t\t\t\t</collision>\n"
    visl_def = "\t\t\t\t<visual name='visual'>\n" + geom_def + "\t\t\t\t</visual>\n"
    link_def = "\t\t\t<link name='link'>\n" + coll_def + visl_def + "\t\t\t</link>\n"
    modl_def = "\n\t\t<model name='" + model_name + "'>\n" + stat_def + pose_def + link_def + "\t\t</model>\n"

    worldfile.write(modl_def)

# Function writes code for a marker in world file
def code_path_marker(obst_loc):

    global mark_cnt
    mark_cnt += 1
    model_name = 'marker_' + str(mark_cnt)
    dimn_def = "\t\t\t\t\t\t\t<radius>0.1</radius>\n\t\t\t\t\t\t\t<length>0.001</length>\n"
    geom_def = "\t\t\t\t\t<geometry>\n\t\t\t\t\t\t<cylinder>\n" + dimn_def + "\t\t\t\t\t\t</cylinder>\n\t\t\t\t\t</geometry>\n"

    stat_def = "\t\t\t<static>1</static>\n"
    pose_def = "\t\t\t<pose>" + str(-obst_loc[0]) + " " + str(-obst_loc[1]) + " 0 0 0 0</pose>\n"
    visl_def = "\t\t\t\t<visual name='visual'>\n" + geom_def + "\t\t\t\t</visual>\n"
    link_def = "\t\t\t<link name='link'>\n" + visl_def + "\t\t\t</link>\n"
    mark_def = "\n\t\t<model name='" + model_name + "'>\n" + stat_def + pose_def + link_def + "\t\t</model>\n"

    worldfile.write(mark_def)

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
    spline_npnt = scintr.splev(np.linspace(0,1,num=25), spline_tck)

    return sp_xcoords, sp_ycoords, spline_npnt

# Function follows gradient
def move_towards_gradient(sum_dt_image, start_pnt, end_pnt, k):
    global obst_img    
    curr_xy = start_pnt
    min_val = np.inf
    min_xy = None
    dt_path = []
    similar_x = []
    im_shape = (int(sum_dt_image.shape[0]), int(sum_dt_image.shape[1]))
    nhood = [(-1,-1),(-1,0),(-1,1),(0,-1),(0,1),(1,-1),(1,0),(1,1)]
    
    while curr_xy != (k*end_pnt[0]-1, k*end_pnt[1]-1):
        for direc in nhood:
            neighbr = tuple(sum(x) for x in zip(curr_xy, direc))
            if neighbr[0] > 0 and neighbr[1] > 0 and neighbr[0] < im_shape[0] and neighbr[1] < im_shape[1]:
                if sum_dt_image[neighbr] < min_val:
                    min_val = sum_dt_image[neighbr]
                    min_xy = neighbr

        if curr_xy == min_xy:
            print "Stuck in a loop"
            return []

        curr_xy = min_xy
        obst_img[curr_xy] = 0
        #print 'Moved to', curr_xy

        if curr_xy[0]%k == 0:
            new_x = float(curr_xy[0])/k; new_y = float(curr_xy[1])/k;
            if len(dt_path) == 0 or (abs(new_x - dt_path[-1][0]) + abs(new_y - dt_path[-1][0])) > 0.8:
                dt_path.append((new_x, new_y))
        
    dt_path.append((15.0,15.0))
    dt_path.insert(0,(0.0,0.0))
    return dt_path

# Function generates path by Voronoi Graph Method
def generate_voronoi_path(obst_img, coords, start_pnt, end_pnt, k):
    edge_img, sep_coords, arr_coords = generate_voronoi_data(obst_img, coords, k)
    vor, graph, start_node, end_node = create_voronoi_graph(arr_coords, start_pnt, end_pnt)
    sp_xcoords, sp_ycoords, spline_npnt = find_best_path_and_spline(vor, graph, start_node, end_node)

    #plt.scatter(sep_coords[0], sep_coords[1])
    #plt.plot(sp_xcoords, sp_ycoords, 'ro')
    #plt.plot(spline_npnt[0], spline_npnt[1], 'r-')
    #plt.show()

    vg_path = []
    for i in range(0, len(spline_npnt[0])):
        vg_path.append((round(spline_npnt[0][i],1), round(spline_npnt[1][i],1)))

    return vg_path

# Function generates path by Distance Transform Method
def generate_dt_path(obst_img, start_pnt, end_pnt, k):
    for x in [0, int(obst_img.shape[0]-1)]:
        for y in range(0, int(obst_img.shape[1])):
            obst_img[x,y] = 0
            obst_img[y,x] = 0

    dest_img = np.ones([k*end_pnt[0], k*end_pnt[1]])
    dest_img[(k*end_pnt[0]-1, k*end_pnt[1]-1)] = 0
    dest_dt_img = scndim.morphology.distance_transform_edt(dest_img)
    obst_dt_img = scndim.morphology.distance_transform_edt(obst_img)
    sum_dt_img = dest_dt_img - obst_dt_img

    dt_path = move_towards_gradient(sum_dt_img, start_pnt, end_pnt, k)

    #io.imshow(sum_dt_img)
    #pil_img = pil.Image.fromarray(255*obst_img)
    #pil_img.show()

    return dt_path

# Define necesary variables
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
          'peri_const' : 2}
k = 10
start_pnt = (0,0)
end_pnt = (15,15)
coords = [start_pnt, end_pnt]
cyl_cnt = 0
cub_cnt = 0
mark_cnt = 0

# Generate random obstacles
obst_img, obstacles = generate_obstacle_map(start_pnt, end_pnt, k)

# Generate path by voronoi graph method and save to file
vg_path = generate_voronoi_path(obst_img, coords, start_pnt, end_pnt, k)
vg_path_file = open('vg_path','w')

for row in vg_path:
    vg_path_file.write(str(row[0]) + '\t' + str(row[1]) + '\n')

vg_path_file.close()
print 'Length of Voronoi Path =', len(vg_path)

# Generate path by distance transform method and save to file
dt_path = generate_dt_path(obst_img, start_pnt, end_pnt, k)
dt_path_file = open('dt_path','w')

for row in dt_path:
    dt_path_file.write(str(row[0]) + '\t' + str(row[1]) + '\n')

dt_path_file.close()
print 'Length of DT Path =', len(dt_path)

# Write code for Gazebo world file
# Codes random obstacles and path markers
path_to_world_file = '/home/chaitanya_pb/catkin_ws/src/snake_robot/worlds/'
worldfile = open(path_to_world_file + 'vg_random_obstacle.world', 'w')

code_initial()
for obst in obstacles:
    code_obstacle(obst[1], obst[2], obst[3])
for mark in vg_path:
    code_path_marker(mark)
code_ending()

worldfile.close()

cyl_cnt = 0
cub_cnt = 0

worldfile = open(path_to_world_file + 'dt_random_obstacle.world', 'w')

code_initial()
for obst in obstacles:
    code_obstacle(obst[1], obst[2], obst[3])
for mark in dt_path:
    code_path_marker(mark)
code_ending()

worldfile.close()

print len(obstacles), 'obstacles:', cyl_cnt, 'cylinders and', cub_cnt, 'cuboids created'
