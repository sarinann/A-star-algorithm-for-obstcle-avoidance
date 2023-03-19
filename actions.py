import numpy as np
import heapq

def actions_to_take (d, theta):
    actions = [[d * np.cos(np.deg2rad(theta+0)), d * np.sin(np.deg2rad(theta+0)), (theta+0)],
               [d * np.cos(np.deg2rad(theta+30)), d * np.sin(np.deg2rad(theta+30)), (theta+30)],
               [d * np.cos(np.deg2rad(theta-30)), d * np.sin(np.deg2rad(theta-30)), (theta-30)],
               [d * np.cos(np.deg2rad(theta+60)), d * np.sin(np.deg2rad(theta+60)), (theta+60)],
               [d * np.cos(np.deg2rad(theta-60)), d * np.sin(np.deg2rad(theta-60)), (theta-60)]]
    return actions

def astar_algorithm(start_node, goal_node, clearance, robot_radius, step_size):

    d_ini_c2c = np.sqrt((goal_node[0]-start_node[0])**2 + (goal_node[1]-start_node[1])**2 )

    start_pose = (d_ini_c2c, start_node, None)
    goal_pose = (0, goal_node, None)

    theta_ini = 30

    open_list = []
    open_list.append(start_pose)

    close_list = []

    lowest_c2c = []
    lowest_c2c.append(start_pose)

    while len(open_list) > 0:

        five_small_nodes = heapq.nsmallest(5, open_list)
        
        for i in five_small_nodes:
            current_node = i

            theta = current_node[1][2]
            close_list.append(current_node)

            distance_to_goal = np.sqrt((goal_node[0] - current_node[0]**2) + (goal_node[1] - current_node[1]**2))

            if distance_to_goal <= step_size*1.5:
                print("Goal has been reached!!!!")

            





