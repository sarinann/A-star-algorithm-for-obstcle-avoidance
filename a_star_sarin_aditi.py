#Git Hub Link: https://github.com/sarinann/A-star-algorithm-for-obstcle-avoidance.git

import matplotlib.pyplot as plt
import matplotlib.patches as patch
from queue import PriorityQueue
import time
import copy

start_time = time.time()
#Defining the four walls. The point should lie within the boundary
def Main_Wall(x, y):
    if (x <= 5) or (x >= 595) or (y <= 5) or (y >= 245):
        return True
    else:
        return False
#Creating and defining obstacle spaces
def Rectangle_Top(x, y):
    if (x >= 95) and (x <= 155) and (y >= 145) and (y <= 250):
        return True
    else:
        return False

def Rectangle_Bottom(x, y):
    if (x >= 95) and (x <= 155) and (y >= 0) and (y <= 105):
        return True
    else:
        return False

def Triangle(x, y):
    if (x>= 455) and (((y-240)-(((240-125)/(455-515))*(x-455)))<=0) and (((y-125)-(((125-10)/(515-455))*(x-515)))>=0):
        return True
    else:
        return False

def Hexagon(x, y):
    if (x >= 230.05) and (x <= 369.95) and (((y-84.61)-((84.61-44.22)/(369.95-300))*(x-369.95)) >= 0) and (((y-44.22)-((44.22-84.61)/(300-230.04))*(x-300)) >= 0) and (((y-165.38)-((165.38-205.77)/(230.05-300))*(x-230.05)) <= 0) and (((y-205.77)-((205.77-165.38)/(300-369.95))*(x-300)) <= 0):
        return True
    else:
        return False

def Obstacle_space(x, y):
    if (Main_Wall(x, y) or Rectangle_Top(x, y) or Rectangle_Bottom(x, y) or Triangle(x, y) or Hexagon(x, y)) == True:
        return True
    else:
        return False

def dijkstra_node_create(cost, parent, node):
    return (cost, parent, node)

# Defining the movements: up. left. right, down, up left, up right, down left and down right   
def ActionMove_neg60(node):
    modified_node = copy.deepcopy(node)
    cost = modified_node[0]+1
    parent_node = modified_node[2]
    x_coord = modified_node[2][0]   
    y_coord = modified_node[2][1]+1
    modified_node = (x_coord, y_coord)
    passed_node = dijkstra_node_create(cost, parent_node, modified_node)
    return passed_node

def ActionMove_neg30(node):
    modified_node = copy.deepcopy(node)
    cost = modified_node[0]+1
    parent_node = modified_node[2]
    x_coord = modified_node[2][0]   
    y_coord =modified_node[2][1]-1
    modified_node = (x_coord, y_coord)
    passed_node = dijkstra_node_create(cost, parent_node, modified_node)
    return passed_node

def ActionMove_zero(node):
    modified_node = copy.deepcopy(node)
    cost = modified_node[0]+1
    parent_node = modified_node[2]
    x_coord = modified_node[2][0]-1
    y_coord =modified_node[2][1]
    modified_node = (x_coord, y_coord)
    passed_node = dijkstra_node_create(cost, parent_node, modified_node)
    return passed_node

def ActionMove_pos30(node):
    modified_node = copy.deepcopy(node)
    cost = modified_node[0]+1
    parent_node = modified_node[2]
    x_coord = modified_node[2][0]+1   
    y_coord =modified_node[2][1]
    modified_node = (x_coord, y_coord)
    passed_node = dijkstra_node_create(cost, parent_node, modified_node)
    return passed_node
    
def ActionMove_pos60(node):
    modified_node = copy.deepcopy(node)
    cost = modified_node[0]+1.4
    parent_node = modified_node[2]
    x_coord = modified_node[2][0]-1   
    y_coord =modified_node[2][1]+1
    modified_node = (x_coord, y_coord)
    passed_node = dijkstra_node_create(cost, parent_node, modified_node)
    return passed_node

start_point_x = input("Enter the x-coordinate of the start point \n")
start_point_y = input("Enter the y-coordinate of the start point \n")
start_orien = input("Define the initial orientation in degrees - Options: -60,-30, 0, 30, 60")
step_size = input("Enter the step size for the robot. Choose from 1 to 10")
start= (int(start_point_x), int(start_point_y), float(start_orien))
while Obstacle_space(start[0], start[1], start[2]):
    print("These coordinates lie inside the obstacle space. Please enter new values\n")
    start_point_x = input("Enter the x-coordinate of the start point \n")
    start_point_y = input("Enter the y-coordinate of the start point \n")
    start= (int(start_point_x), int(start_point_y))

goal_point_x = input("Enter the x-coordinate of the goal point \n")
goal_point_y = input("Enter the y-coordinate of the goal point \n")
goal_orien = input("Define the final orientation in degrees - Options: -60,-30, 0, 30, 60")
goal= (int(goal_point_x), int(goal_point_y), float(goal_orien))
while Obstacle_space(goal[0], goal[1]):
    print("These coordinates lie inside the obstacle space. Please enter new values\n")
    goal_point_x = input("Enter the x-coordinate of the goal point \n")
    goal_point_y = input("Enter the y-coordinate of the goal point \n")
    goal= (int(goal_point_x), int(goal_point_y))