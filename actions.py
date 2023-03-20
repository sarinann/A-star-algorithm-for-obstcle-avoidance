import numpy as np
import heapq
from a_star_sarin_aditi import *

def actions_to_take (d, theta):
    actions = [[d * np.cos(np.deg2rad(theta+0)), d * np.sin(np.deg2rad(theta+0)), (theta+0)],
               [d * np.cos(np.deg2rad(theta+30)), d * np.sin(np.deg2rad(theta+30)), (theta+30)],
               [d * np.cos(np.deg2rad(theta-30)), d * np.sin(np.deg2rad(theta-30)), (theta-30)],
               [d * np.cos(np.deg2rad(theta+60)), d * np.sin(np.deg2rad(theta+60)), (theta+60)],
               [d * np.cos(np.deg2rad(theta-60)), d * np.sin(np.deg2rad(theta-60)), (theta-60)]]
    return actions

def astar_algorithm(start_node, goal_node, clearance, robot_radius, step_size, thre):

    d_ini_c2c = np.sqrt((goal_node[0]-start_node[0])**2 + (goal_node[1]-start_node[1])**2 )

    zeros = np.zeros((int(400/thre),int(250/thre),10))
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

            distance_to_goal = ((goal_node[0] - current_node[0])**2 + (goal_node[1] - current_node[1])**2)

            if distance_to_goal <= (step_size*1.5)**2:
                print("Goal has been reached!!!!")

            nodes_to_explore = 5
            for node in range(nodes_to_explore):

                generate_new_node = actions_to_take(step_size, theta_ini)

                node_pose = ((current_node[1][0] + generate_new_node[node][0]) , (current_node[1][1] + generate_new_node[node][1]), generate_new_node[node][2])

                parent_node = current_node[2]

                c2c = np.sqrt((goal_node[0] - node_pose[0])**2 - (goal_node[1] - node_pose[1])**2)

                c2g = np.sqrt((parent_node[0] - node_pose[0])**2 - (parent_node[1] - node_pose[1])**2)

                total_cost = int(c2c + c2g)

                nodeok = Obstacle_space(node_pose[0], node_pose[1], node_pose[2])

                if nodeok == False:
                    if  zeros[int(node_pose[0]/thre)][int(node_pose[1]/thre)][int(node_pose[2]/30)]==0:
                        zeros[int(node_pose[0]/thre)][int(node_pose[1]/thre)][int(node_pose[2]/30)]=1
                        final_new_node = (total_cost, node_pose, parent_node)
                        heapq.heappush(open_list, final_new_node)
                    else:
                        continue

        heapq.heappop(open_list)     

path = astar_algorithm(start, goal, 5, 5, step_size, 5)

print(path)
  

                










            





