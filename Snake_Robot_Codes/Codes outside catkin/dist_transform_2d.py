# Author: Chaitanya Pb

# This code creates a random obstacle map. It uses the Distance Transform Method to automatically 
# generate a path through the created obstacle field and displays the resultant path.

# Standalone: It does not call functions from any other file.

# Doesn't contain paths to external files.

import numpy as np
import random as rnd
import scipy.ndimage as scndim
import PIL as pil
from skimage import io, filter, morphology, feature
import matplotlib as plt

# Function generates a random obstacle map
def generate_obstacle_map(start_pnt, end_pnt):
    
    obstacles = []
    coords = [start_pnt, end_pnt]
    obst_image = np.ones([100*end_pnt[0], 100*end_pnt[1]])
    
    obst_num = rnd.randint(3, c_dict['max_obst'])

    for obst_id in range(1, obst_num+1):
        obst_type = rnd.randint(1, len(c_dict['name_types']))
        obst_xy = (rnd.uniform(c_dict['x_min'], c_dict['x_max']), rnd.uniform(c_dict['y_min'], c_dict['y_max']))
        obst_image_xy = tuple(int(round(100*element)) for element in obst_xy)
        
        if c_dict['name_types'][obst_type-1] == 'cylinder':    
            obst_dim = [rnd.uniform(c_dict['r_min'], c_dict['r_max'])]
            obst_image_dim = tuple(int(round(100*element)) for element in obst_dim)
            for x in range(-obst_image_dim[0], obst_image_dim[0]):
                for y in range(-obst_image_dim[0], obst_image_dim[0]):
                    if x*x + y*y <= obst_image_dim[0]*obst_image_dim[0]:
                        obst_image[obst_image_xy[0]+x, obst_image_xy[1]+y] = 0
                    
        elif c_dict['name_types'][obst_type-1] == 'cuboid':
            obst_len = rnd.uniform(c_dict['l_min'], c_dict['l_max'])
            obst_dim = [obst_len, c_dict['peri_const']-obst_len, rnd.uniform(c_dict['a_min'], c_dict['a_max'])]
            obst_image_dim = tuple(int(round(100*element)) for element in obst_dim)
            for x in range(int(-obst_image_dim[0]/2), int(obst_image_dim[0]/2)):
                for y in range(int(-obst_image_dim[1]/2), int(obst_image_dim[1]/2)):
                    obst_image[obst_image_xy[0]+x, obst_image_xy[1]+y] = 0
                
        coords.append(obst_xy)
        obstacles.append([obst_id, c_dict['name_types'][obst_type-1], obst_xy, obst_dim])

    for x in [0, int(obst_image.shape[0]-1)]:
            for y in range(0, int(obst_image.shape[1])):
                obst_image[x,y] = 0
                obst_image[y,x] = 0
        
    return obst_image, obstacles, coords

# Function follows gradient
def move_towards_gradient(sum_dt_image, start_pnt, end_pnt):
    global obst_img    
    curr_xy = start_pnt
    min_val = np.inf
    min_xy = None
    im_shape = (int(sum_dt_image.shape[0]), int(sum_dt_image.shape[1]))
    nhood = [(-1,-1),(-1,0),(-1,1),(0,-1),(0,1),(1,-1),(1,0),(1,1)]
    
    while curr_xy != (100*end_pnt[0]-1, 100*end_pnt[1]-1):
        for direc in nhood:
            neighbr = tuple(sum(x) for x in zip(curr_xy, direc))
            if neighbr[0] > 0 and neighbr[1] > 0 and neighbr[0] < im_shape[0] and neighbr[1] < im_shape[1]:
                if sum_dt_image[neighbr] < min_val:
                    min_val = sum_dt_image[neighbr]
                    min_xy = neighbr
        
        if curr_xy == min_xy:
        	print "Stuck in a loop"
        	return

        curr_xy = min_xy
        obst_img[curr_xy] = 0
        print 'Moved to', curr_xy
        
    return

# Define necessary variables
c_dict = {'max_obst' : 10,
              'name_types' : ['cylinder', 'cuboid'],
              'x_min' : 3,
              'x_max' : 12,
              'y_min' : 3,
              'y_max' : 12,
              'r_min' : 0.1,
              'r_max' : 0.9,
              'l_min' : 0.1,
              'l_max' : 1.9,
              'a_min' : 0,
              'a_max' : 180,
              'peri_const' : 2,
              'edge_thr' : 0.1}

start_pnt = (0,0)
end_pnt = (15,15)

# Generate random obstacles and automated path
obst_img, obstacles, coords = generate_obstacle_map(start_pnt, end_pnt)

dest_img = np.ones([100*end_pnt[0], 100*end_pnt[1]])
dest_img[(100*end_pnt[0]-1, 100*end_pnt[1]-1)] = 0
dest_dt_img = scndim.morphology.distance_transform_edt(dest_img)
obst_dt_img = scndim.morphology.distance_transform_edt(obst_img)
sum_dt_img = dest_dt_img - obst_dt_img

move_towards_gradient(sum_dt_img, start_pnt, end_pnt)

io.imshow(sum_dt_img)
pil_img = pil.Image.fromarray(255*obst_img)
pil_img.show()
