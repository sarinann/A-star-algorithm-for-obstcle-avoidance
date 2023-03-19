import numpy as np

def actions_to_take (d, theta):
    actions = [[d * np.cos(np.deg2rad(theta+0)), d * np.sin(np.deg2rad(theta+0)), (theta+0)],
               [d * np.cos(np.deg2rad(theta+30)), d * np.sin(np.deg2rad(theta+30)), (theta+30)],
               [d * np.cos(np.deg2rad(theta-30)), d * np.sin(np.deg2rad(theta-30)), (theta-30)],
               d * np.cos(np.deg2rad(theta+60)), d * np.sin(np.deg2rad(theta+60)), (theta+60),
               d * np.cos(np.deg2rad(theta-60)), d * np.sin(np.deg2rad(theta-60)), (theta-60)
               ]
    return actions

