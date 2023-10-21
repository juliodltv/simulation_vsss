#!/usr/bin/python3

import sys, rospkg, cv2, numpy as np
sys.dont_write_bytecode = True

colors = {'green': 1, 'pink': [0,2], 'red': 2, 'cyan': [0,1]}

top_blue = np.zeros((800, 800, 3), dtype=np.uint8)
top_yellow = np.zeros((800, 800, 3), dtype=np.uint8)

top_blue[75+300+50:800-75,75:800-75,0] = 255 # Blue
top_yellow[75+300+50:800-75,75:800-75,1:] = 255 # Yellow

# green-red, cyan-red, red-green, cyan-green, pink-green, red-cyan, green-cyan, pink-cyan, green-pink, cyan-pink

combinations_yellow = [
    
    # Consider cyan as default color
    # cyan at right
    [colors['green'], colors['cyan']],
    [colors['pink'], colors['cyan']],
    [colors['red'], colors['cyan']],
    
    # # cyan at left
    # [colors['cyan'], colors['green']],
    # [colors['cyan'], colors['pink']],
    # [colors['cyan'], colors['red']],
    
    # # Consider green as default color
    # # green at right
    # [colors['cyan'], colors['green']],
    # [colors['pink'], colors['green']],
    # [colors['red'], colors['green']],
    
    # # green at left
    # [colors['green'], colors['cyan']],
    # [colors['green'], colors['pink']],
    # [colors['green'], colors['red']],
    
]

combinations_blue = [
    [colors['cyan'], colors['green']],
    [colors['pink'], colors['green']],
    [colors['red'], colors['green']],
]
# print(len(combinations_blue))

for i in range(len(combinations_blue)):
    # print(combination)
    top_blue[75:75+300,75:75+300,combinations_blue[i][0]] = 255
    top_blue[75:75+300,75+300+50:800-75,combinations_blue[i][1]] = 255
    
    top_yellow[75:75+300,75:75+300,combinations_yellow[i][0]] = 255
    top_yellow[75:75+300,75+300+50:800-75,combinations_yellow[i][1]] = 255
    
    cv2.imwrite(rospkg.RosPack().get_path('simulation_vsss')+f'/media/materials/textures/blue_{i}.png', top_blue)
    cv2.imwrite(rospkg.RosPack().get_path('simulation_vsss')+f'/media/materials/textures/yellow_{i}.png', top_yellow)
    
    top_blue[75:75+300,75:75+300,combinations_blue[i][0]] = 0
    top_blue[75:75+300,75+300+50:800-75,combinations_blue[i][1]] = 0
    
    top_yellow[75:75+300,75:75+300,combinations_yellow[i][0]] = 0
    top_yellow[75:75+300,75+300+50:800-75,combinations_yellow[i][1]] = 0