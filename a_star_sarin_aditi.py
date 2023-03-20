#Git Hub Link: https://github.com/sarinann/A-star-algorithm-for-obstcle-avoidance.git

import matplotlib.pyplot as plt
import matplotlib.patches as patch
from queue import PriorityQueue
import time
import copy
import numpy as np
import heapq
from math import dist

start_time = time.time()
#Defining the four walls. The point should lie within the boundary
def Main_Wall(x, y, theta):
    if (x <= 5) or (x >= 595) or (y <= 5) or (y >= 245):
        return True
    else:
        return False
#Creating and defining obstacle spaces
def Rectangle_Top(x, y, theta):
    if (x >= 95) and (x <= 155) and (y >= 145) and (y <= 250):
        return True
    else:
        return False

def Rectangle_Bottom(x, y, theta):
    if (x >= 95) and (x <= 155) and (y >= 0) and (y <= 105):
        return True
    else:
        return False

def Triangle(x, y, theta):
    if (x>= 455) and (((y-240)-(((240-125)/(455-515))*(x-455)))<=0) and (((y-125)-(((125-10)/(515-455))*(x-515)))>=0):
        return True
    else:
        return False

def Hexagon(x, y, theta):
    if (x >= 230.05) and (x <= 369.95) and (((y-84.61)-((84.61-44.22)/(369.95-300))*(x-369.95)) >= 0) and (((y-44.22)-((44.22-84.61)/(300-230.04))*(x-300)) >= 0) and (((y-165.38)-((165.38-205.77)/(230.05-300))*(x-230.05)) <= 0) and (((y-205.77)-((205.77-165.38)/(300-369.95))*(x-300)) <= 0):
        return True
    else:
        return False

def Obstacle_space(x, y, theta):
    if (Main_Wall(x, y, theta) or Rectangle_Top(x, y, theta) or Rectangle_Bottom(x, y, theta) or Triangle(x, y, theta) or Hexagon(x, y, theta)) == True:
        return True
    else:
        return False

def dijkstra_node_create(cost, cost_to_come, parent, node):
    return (cost,cost_to_come, parent, node)


step_size = 5
# Defining the movements: up. left. right, down, up left, up right, down left and down right   
def ActionMove_neg60(node):
    modified_node = copy.deepcopy(node)
    cost = modified_node[1]+1
    parent_node = modified_node[3]

    modi_theta = (parent_node[2] - 60) % 360
    modi_x = parent_node[0] + (step_size*np.cos(np.radians(modi_theta)))
    modi_y = parent_node[1] + (step_size*np.sin(np.radians(modi_theta)))

    modified_node = (modi_x, modi_y, modi_theta)

    cost_to_go = dist((modi_x, modi_x), (goal_x, goal_y))

    total_cost = cost + cost_to_go

    passed_node = dijkstra_node_create(total_cost, cost, parent_node, modified_node)
    return passed_node

def ActionMove_neg30(node):
    modified_node = copy.deepcopy(node)
    cost = modified_node[1]+1
    parent_node = modified_node[3]

    modi_theta = (parent_node[2] - 30) % 360
    modi_x = parent_node[0] + (step_size*np.cos(np.radians(modi_theta)))
    modi_y = parent_node[1] + (step_size*np.sin(np.radians(modi_theta)))

    modified_node = (modi_x, modi_y, modi_theta)

    cost_to_go = dist((modi_x, modi_x), (goal_x, goal_y))

    total_cost = cost + cost_to_go

    passed_node = dijkstra_node_create(total_cost, cost, parent_node, modified_node)
    return passed_node

def ActionMove_zero(node):
    modified_node = copy.deepcopy(node)
    cost = modified_node[1]+1
    parent_node = modified_node[3]

    modi_theta = (parent_node[2]) % 360
    modi_x = parent_node[0] + (step_size*np.cos(np.radians(modi_theta)))
    modi_y = parent_node[1] + (step_size*np.sin(np.radians(modi_theta)))

    modified_node = (modi_x, modi_y, modi_theta)

    cost_to_go = dist((modi_x, modi_x), (goal_x, goal_y))

    total_cost = cost + cost_to_go

    passed_node = dijkstra_node_create(total_cost, cost, parent_node, modified_node)
    return passed_node


def ActionMove_pos30(node):
    modified_node = copy.deepcopy(node)
    cost = modified_node[1]+1
    parent_node = modified_node[3]

    modi_theta = (parent_node[2] + 30) %360
    modi_x = parent_node[0] + (step_size*np.cos(np.radians(modi_theta)))
    modi_y = parent_node[1] + (step_size*np.sin(np.radians(modi_theta)))

    modified_node = (modi_x, modi_y, modi_theta)

    cost_to_go = dist((modi_x, modi_x), (goal_x, goal_y))

    total_cost = cost + cost_to_go

    passed_node = dijkstra_node_create(total_cost, cost, parent_node, modified_node)
    return passed_node

    
def ActionMove_pos60(node):
    modified_node = copy.deepcopy(node)
    cost = modified_node[1]+1
    parent_node = modified_node[3] + 60

    modi_theta = (parent_node[2]) % 360
    modi_x = parent_node[0] + (step_size*np.cos(np.radians(modi_theta)))
    modi_y = parent_node[1] + (step_size*np.sin(np.radians(modi_theta)))

    modified_node = (modi_x, modi_y, modi_theta)

    cost_to_go = dist((modi_x, modi_x), (goal_x, goal_y))

    total_cost = cost + cost_to_go

    passed_node = dijkstra_node_create(total_cost, cost, parent_node, modified_node)
    return passed_node




# def move_plus_thirty(node, step_size, theta):

#     action = (step_size* np.cos(np.deg2rad(theta+30)), step_size* np.sin(np.deg2rad(theta+30)), (theta+30))

#     modified_node = copy.deepcopy(node)

# start_point_x = input("Enter the x-coordinate of the start point \n")
# start_point_y = input("Enter the y-coordinate of the start point \n")
# start_orien = input("Define the initial orientation in degrees - Options: -60,-30, 0, 30, 60")
# step_size = input("Enter the step size for the robot. Choose from 1 to 10")
# start= (int(start_point_x), int(start_point_y), float(start_orien))
# print(type(start))
# while Obstacle_space(start[0], start[1], start[2]):
#     print("These coordinates lie inside the obstacle space. Please enter new values\n")
#     start_point_x = input("Enter the x-coordinate of the start point \n")
#     start_point_y = input("Enter the y-coordinate of the start point \n")
#     start_orien = input("Define the initial orientation in degrees - Options: -60,-30, 0, 30, 60")
#     start= (int(start_point_x), int(start_point_y), float(start_orien))

# goal_point_x = input("Enter the x-coordinate of the goal point \n")
# goal_point_y = input("Enter the y-coordinate of the goal point \n")
# goal_orien = input("Define the final orientation in degrees - Options: -60,-30, 0, 30, 60")
# goal= (int(goal_point_x), int(goal_point_y), float(goal_orien))
# print(type(goal))
start = (10, 10, 0)

goal = (10, 20, 0)

step = 5


while Obstacle_space(goal[0], goal[1], goal[2]):
    print("These coordinates lie inside the obstacle space. Please enter new values\n")
    goal_point_x = input("Enter the x-coordinate of the goal point \n")
    goal_point_y = input("Enter the y-coordinate of the goal point \n")
    goal_orien = input("Define the final orientation in degrees - Options: -60,-30, 0, 30, 60")
    goal= (int(goal_point_x), int(goal_point_y), float(goal_orien))

# def actions_to_take (step_size, theta):
#     actions = [[step_size* np.cos(np.deg2rad(theta+0)), step_size* np.sin(np.deg2rad(theta+0)), (theta+0)],
#                [step_size* np.cos(np.deg2rad(theta+30)), step_size* np.sin(np.deg2rad(theta+30)), (theta+30)],
#                [step_size* np.cos(np.deg2rad(theta-30)), step_size* np.sin(np.deg2rad(theta-30)), (theta-30)],
#                [step_size* np.cos(np.deg2rad(theta+60)), step_size* np.sin(np.deg2rad(theta+60)), (theta+60)],
#                [step_size* np.cos(np.deg2rad(theta-60)), step_size* np.sin(np.deg2rad(theta-60)), (theta-60)]]
#     return actions

# def astar_algorithm(start_node, goal_node, clearance, robot_radius, step_size1, thre):

#     d_ini_c2g = np.sqrt((goal_node[0]-start_node[0])**2 + (goal_node[1]-start_node[1])**2 )
#     # c2c = 0
#     zeros = np.zeros((int(400/thre),int(250/thre),10))

#     start_pose = (d_ini_c2g, start_node, None)
#     goal_pose = (0, goal_node, None)

#     theta_ini = 0

#     five_small_nodes = []
#     five_small_nodes.append(start_pose)

#     open_list = []
#     open_list.append(start_pose)

#     close_list = []

#     # lowest_c2c = []
#     # lowest_c2c.append(start_pose)

#     while len(open_list) > 0:

#         five_small_nodes = heapq.nsmallest(1, open_list)

#         for i in five_small_nodes:

#             current_node = i

#             theta = current_node[1][2]
#             close_list.append(current_node)

#             # print(type(goal_node))
#             # print(type(current_node))

#             distance_to_goal = np.sqrt((goal_node[0] - current_node[1][0])**2 + (goal_node[1] - current_node[1][1])**2)

#             if distance_to_goal <= int(5*1.5):
#                 print("Goal has been reached!!!!")

#             nodes_to_explore = 5

#             for node in range(nodes_to_explore):

#                 generate_new_node = actions_to_take(5, theta)

#                 node_pose = ((current_node[1][0] + generate_new_node[node][0]) , (current_node[1][1] + generate_new_node[node][1]), (generate_new_node[node][2]))

#                 parent_node = current_node[2]

#                 c2g = np.sqrt((goal_node[0] - node_pose[0])**2 - (goal_node[1] - node_pose[1])**2)

#                 if parent_node is None:
#                     c2c = 0
#                     total_cost = c2g
#                 else:

#                     c2c = np.sqrt((parent_node[0] - node_pose[0])**2 - (parent_node[1] - node_pose[1])**2)
#                     total_cost = c2c + c2g


#                 # total_cost = int(c2c + c2g)

#                 nodeok = Obstacle_space(node_pose[0], node_pose[1], node_pose[2])

#                 if nodeok == False:
#                     if  zeros[int(node_pose[0]/thre)][int(node_pose[1]/thre)][int(node_pose[2]/30)]==0:
#                         zeros[int(node_pose[0]/thre)][int(node_pose[1]/thre)][int(node_pose[2]/30)]=1
#                         final_new_node = (total_cost, node_pose, parent_node)
#                     # if node_pose not in close_list:

#                         heapq.heappush(open_list, final_new_node)
#                     else:
#                         continue

#         heapq.heappop(open_list)   
#     return close_list    

# path = astar_algorithm(start, goal, 5, 5, 5, 5)

# print(path)


def Astar_algoritm(start, goal, step, clearance, robot_radius):

    initial_cost = np.sqrt((goal[0]-start[0])**2 + (goal[1]-start[1])**2 )
    
    start_pose = (initial_cost, start, None)
    goal_pose = (0, goal, None)

    # theta_ini = 0

    open_list = PriorityQueue()
    open_list.put(start_pose)

    close_list = {}

    while not open_list.empty():

        current_node = open_list.get()

        modi_current_node = copy.deepcopy(current_node)

        if modi_current_node[1] in close_list:
            continue

        close_list[modi_current_node[1]] = modi_current_node[2]

        theta = modi_current_node[1][2]

        distance_to_goal = np.sqrt((goal[0] - modi_current_node[1][0])**2 + (goal[1] - modi_current_node[1][1])**2)

        if distance_to_goal <= int(5*1.5):
            print("Goal has been reached!!!!")

        nodes_to_explore = 5

        for node in range(nodes_to_explore):

            present_node = actions_to_take(step, theta)

            node_pose = ((modi_current_node[1][0] + present_node[node][0]) , (modi_current_node[1][1] + present_node[node][1]), (present_node[node][2]))

            node_ok = Obstacle_space(node_pose[0], node_pose[1], node_pose[2])

            if 

            






