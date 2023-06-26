#!/usr/bin/python3

import sys, rospkg, cv2, numpy as np
sys.dont_write_bytecode = True

offset = 50
colors = {'green': 1, 'pink': [0,2], 'red': 2}

top_blue = np.zeros((897, 897, 3), dtype=np.uint8)
top_yellow = np.zeros((897, 897, 3), dtype=np.uint8)

top_blue[897//2:897-offset,offset:897-offset,0] = 255 # Blue
top_yellow[897//2:897-offset,offset:897-offset,1:] = 255 # Yellow

for i, value in enumerate(colors.values()):
    top_blue[offset:897//2,offset:897-offset,value] = 255
    top_yellow[offset:897//2,offset:897-offset,value] = 255

    cv2.imwrite(rospkg.RosPack().get_path('simulation_vsss')+f'/media/materials/textures/blue_{i}.png', top_blue)
    cv2.imwrite(rospkg.RosPack().get_path('simulation_vsss')+f'/media/materials/textures/yellow_{i}.png', top_yellow)
    
    top_blue[offset:897//2,offset:897-offset,value] = 0
    top_yellow[offset:897//2,offset:897-offset,value] = 0