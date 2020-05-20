import os
import numpy as np
from matplotlib import pyplot as plt


def get_drawable_map(map, target_x = 200, target_y = 200):

    color_unknown = -80
    color_known = 0
    color_obstacle = 150
    color_target = 200

    target_dim = 4
    target_x_index = slice(target_x - (target_dim//2), target_x + (target_dim//2))
    target_y_index = slice(target_y - (target_dim//2), target_y + (target_dim//2))

    drawable_map = np.flipud(map)
    drawable_map[drawable_map == -1] = color_unknown # recoloring unknown
    drawable_map[drawable_map == 0] = color_known # recoloring known
    drawable_map[drawable_map == 100] = color_obstacle # recoloring obstacles
    drawable_map[target_x_index,target_y_index] = color_target # recoloring target

    return drawable_map



npy_name = 'indoor_1'
script_folder = os.path.dirname(os.path.realpath(__file__))
map_path = script_folder + '/' + npy_name

map = np.load(map_path + '.npy')
print('map shape: ', map.shape)
# np.savetxt(map_path + '.txt', map, '%4.0f')


plt.figure(figsize = (6,6))
plt.imshow(get_drawable_map(map), interpolation='nearest')
plt.pause(20)
