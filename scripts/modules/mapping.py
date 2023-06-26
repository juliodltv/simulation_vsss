import rospkg, numpy as np

USB = {'buttons': ['Y', 'B', 'A', 'X', 'LB', 'RB', 'LT', 'RT', 'BACK', 'START', 'LSB', 'RSB'],
       'axes': ['LH', 'LV', 'RV', 'RH', 'PADH', 'PADV']}

Xbox360 = {'buttons': ['A', 'B', 'X', 'Y', 'LB', 'RB', 'BACK', 'START', 'LSB', 'RSB'],
           'axes': ['LH', 'LV', 'LT', 'RH', 'RV', 'RT', 'PADH', 'PADV']}

XboxOne = {'buttons': ['A', 'B', 'X', 'Y', 'LB', 'RB', 'BACK', 'START', 'XBOX', 'LSB', 'RSB'],
           'axes': ['LH', 'LV', 'LT', 'RH', 'RV', 'RT', 'PADH', 'PADV']}

np.savez(rospkg.RosPack().get_path('simulation_vsss')+'/scripts/modules/joysticks.npz', USB=USB, Xbox360=Xbox360, XboxOne=XboxOne)