# Git Hub Link: https://github.com/sarinann/A-star-algorithm-for-obstcle-avoidance.git

import matplotlib.pyplot as plt
import matplotlib.patches as patch
from queue import PriorityQueue
import time
import copy
import numpy as np
from math import dist
import cv2 as cv
import math

print("Hi!! \n")
start_time = time.time()
threshold = 0.5
robot_radius = int(input("Enter the radius of the robot \n"))
print("____________________________________________________________")
clearance = int(input("Enter the clearance to be maintained around the obstacles \n"))
print("______________________________________________________________________")

ul = int(input("Enter the velocity of left wheel \n"))
print("______________________________________________________________________")

ur = int(input("Enter the velocity of right wheel \n"))
print("______________________________________________________________________")

#Total amount the boundary needs to be bloated by taking into account robot radius and clearance.
total_bloat = clearance + robot_radius

# Using OpenCV libraries to create shapes of the polynomials of the obstacles and their map
def create_polynomial(image, vertices, color, type):
    vertices = vertices.reshape((-1,1,2))
    if type == "polygon":
        cv.fillPoly(image,[vertices],color)
    else:
        cv.polylines(image,[vertices],True,color, thickness = total_bloat)

def create_map():
    #initializing the map
    map = np.zeros((250,600, 3), dtype="uint8") 
    white = (255,255,255)
    # Hexagon
    hexagon = np.array([[300,50], [365, 87.5], [365,162.5], 
                       [300,200], [235, 162.5], [235, 87.5]], np.int32)
    create_polynomial(map, hexagon, white, "polygon")
    hexagon_bloat = np.array([[300,45], [369, 87.5], [369,162.5], 
                          [300,205], [235, 162.5], [235, 87.5]], np.int32)
    create_polynomial(map, hexagon_bloat, white, "Border")

    # Triangle
    traingle = np.array([[460, 25], [460, 225], [510,125]], np.int32)
    create_polynomial(map, traingle, white, "polygon")
    triangle_bloat = np.array([[456, 20], [456, 230], [514,125]], np.int32)
    create_polynomial(map, triangle_bloat, white, "Border") 

    # Rectangle
    cv.rectangle(map, (100,0), (150,100), white, -1)
    cv.rectangle(map, (100,0), (150,100), white, total_bloat)
    cv.rectangle(map, (100,150), (150,250), white, -1)
    cv.rectangle(map, (100,150), (150, 250), white, total_bloat)
    cv.rectangle(map, (-1, -1), (601, 251), white, total_bloat)     

    return cv.resize(map, (int(600/threshold),int(250/threshold)))

#Creating the map and flipping it about Y-Axis to match the origin points 
map = create_map()

map = cv.flip(map, 0)

def if_obstacle(node):
    X = int(node[1]/threshold)
    Y = int(node[0]/threshold)
  
    if map[X, Y].any() == np.array([0, 0, 0]).all():
        bool_value = False
    elif map[X, Y].all() == np.array([255, 255, 255]).all():
        bool_value = True
    else:
        bool_value = True     

    return bool_value
#Function to create a node with parameters needed for the algorithm
def a_star_node_create(total_cost, cost_to_come, parent, node):
    return (total_cost, cost_to_come, parent, node)

start_point_x = input("Enter the x-coordinate of the start point \n")
start_point_y = input("Enter the y-coordinate of the start point \n")
start_point_theta = input("Enter the start orientation \n")
start = (int(start_point_x), int(start_point_y), int(start_point_theta))
while if_obstacle((start[0], start[1])):
    print("These coordinates lie inside the obstacle space. Please enter new values\n")
    start_point_x = input("Enter the x-coordinate of the start point \n")
    start_point_y = input("Enter the y-coordinate of the start point \n")
    start_point_theta = input("Enter the start orientation \n")
    start = (int(start_point_x), int(start_point_y), int(start_point_theta))
print("_____________________________________________________________________________")
goal_point_x = input("Enter the x-coordinate of the goal point \n")
goal_point_y = input("Enter the y-coordinate of the goal point \n")
goal_point_orien = input("Enter the goal orientation \n")
goal = (int(goal_point_x), int(goal_point_y), int(goal_point_orien))
while if_obstacle((goal[0], goal[1])):
    print("These coordinates lie inside the obstacle space. Please enter new values\n")
    goal_point_x = input("Enter the x-coordinate of the goal point \n")
    goal_point_y = input("Enter the y-coordinate of the goal point \n")
    goal_point_orien = input("Enter the goal orientation \n")
    goal = (int(goal_point_x), int(goal_point_y), int(goal_point_orien))
print("_____________________________________________________________________________")
goal_x = goal[0]
goal_y = goal[1]


def cost(Xi,Yi,Thetai,UL,UR):
    t = 0
    r = 0.038
    L = 0.354
    dt = 0.1
    Xn=Xi
    Yn=Yi
    Thetan = 3.14 * Thetai / 180


# Xi, Yi,Thetai: Input point's coordinates
# Xs, Ys: Start point coordinates for plot function
# Xn, Yn, Thetan: End point coordintes
    D=0
    while t<1:
        t = t + dt
        Xs = Xn
        Ys = Yn
        Delta_Xn = 0.5*r * (UL + UR) * math.cos(Thetan) * dt
        Delta_Yn = 0.5*r * (UL + UR) * math.sin(Thetan) * dt
        Thetan += (r / L) * (UR - UL) * dt
        D=D+ math.sqrt(Delta_Xn**2 + Delta_Yn**2)
    Thetan = 180 * (Thetan) / 3.14
    return Xn, Yn, Thetan, D   



# Defining the 5 moments spanning 120deg  
def one(node, left_speed, right_speed):
    modified_node = copy.deepcopy(node)
    modi_x, modi_y, modi_theta, c2c = cost(modified_node[3][0], modified_node[3][1], modified_node[3][2], left_speed, right_speed)

    parent_node = modified_node[3]

    modi_theta = rounded_value(modi_theta)
    modi_x = rounded_value(modi_x)
    modi_y = rounded_value(modi_y)
    modified_node = (modi_x, modi_y, modi_theta)

    cost_to_go = dist((modi_x, modi_y), (goal_x, goal_y))

    total_cost = c2c + cost_to_go

    passed_node = a_star_node_create(
        total_cost, c2c, parent_node, modified_node)
    return passed_node

def rounded_value(input):
    if input % 0.5 != 0:
        input = (np.round(input/threshold))*threshold
    return input

plotting = {}
def Astar_algoritm(start, goal):

    initial_cost_to_go = np.sqrt((goal[0]-start[0])**2 + (goal[1]-start[1])**2)

    initial_cost = initial_cost_to_go + 0

    zeros = np.zeros((int(600/threshold), int(250/threshold), 12))

    start_pose = a_star_node_create(initial_cost, 0, None, start)

    open_list = PriorityQueue()
    open_list.put(start_pose)

    visited_nodes_x = []
    visited_nodes_y = []
    visited_close_list = {}

    while not open_list.empty():

        current_node = open_list.get()
        if current_node[3] in visited_close_list:
            continue

        visited_nodes_x.append(current_node[3][0])
        visited_nodes_y.append(current_node[3][1])

        visited_close_list[current_node[3]] = current_node[2]
        theta = current_node[3][2]

        distance_to_goal = np.sqrt(
            (goal[0] - current_node[3][0])**2 + (goal[1] - current_node[3][1])**2)

        if distance_to_goal <= 1.5:
            print("Goal has been reached!!!!")
            print("_____________________________________________________________________________")
            generated_path = []
            current_pose = current_node[3]
            while current_pose is not None:
                generated_path.append(current_pose)
                current_pose = visited_close_list[current_pose]
            # Reverse the path and return
            return generated_path[::-1], visited_nodes_x, visited_nodes_y

        actions = [[0, ul], [ul,0], [ul, ul], [0, ur], [ur, 0], [ur, ur], [ul, ur], [ur, ul]]

        for action in actions:
            # Get the modified node and its total cost and cost to come from current node
            child_node = one(current_node, action[0], action[1])
            temp_node = child_node[3]
            # If the node is not in the the visited closed dictionary, then update the parent based on cost
            if zeros[int(temp_node[0]/threshold)][int(temp_node[1]/threshold)][int(temp_node[2]/30)] == 0:
                zeros[int(temp_node[0]/threshold)][int(temp_node[1]/threshold)
                                            ][int(temp_node[2]/30)] = 1
                if if_obstacle((temp_node[0], temp_node[1])) == False:
                    total_cost = child_node[0]
                    if child_node[3] not in visited_close_list:
                        for i in range(0, (open_list.qsize())):
                            if open_list.queue[i][3] == child_node[3] and open_list.queue[i][0] > total_cost:
                                open_list.queue[i][2] = child_node[2]
                        open_list.put(child_node)
                        visited_nodes_x.append(child_node[3][0])
                        visited_nodes_y.append(child_node[3][1])
    return None


generated_path, x_visited, y_visited = Astar_algoritm(start, goal)
end_time = time.time()
# print(generated_path)

print(
    f'Time taken to solve using the A* algorithm: {end_time - start_time} \n')
print("_____________________________________________________________________________")
path_x_coord = []
path_y_coord = []
for i in range(len(generated_path)):
    path_x_coord.append(generated_path[i][0])
    path_y_coord.append(generated_path[i][1])

fig, ax = plt.subplots(figsize=(6, 2.5))
Triangle = patch.Polygon([(460, 25), (460, 225), (510, 125)], linewidth=1, edgecolor='y', facecolor='y')
Rectangle_Bottom = patch.Rectangle((100, 150), 50, 100, linewidth=1, edgecolor='y', facecolor='y')
Rectangle_Top = patch.Rectangle((100, 0), 50, 100, linewidth=1, edgecolor='y', facecolor='y')
Hexagon = patch.RegularPolygon((300, 125), 6, 75, linewidth=1, edgecolor='y', facecolor='y')
# Plotting the obstacles
ax.add_patch(Triangle)
ax.add_patch(Rectangle_Bottom)
ax.add_patch(Rectangle_Top)
ax.add_patch(Hexagon)

# Plotting the path
plt.xlabel('X-Axis')
plt.ylabel('Y-Axis')
plt.title("Visualizing Explored Nodes through A* Algorithm")
plt.axis([0, 600, 0, 250])

x_temp = 0
y_temp = 0

# for i in range(len(x_visited)):
#     if x_temp == goal[0] and y_temp == goal[1]:
#         break
#     if len(x_visited) > 100:
#         plt.scatter(x_visited[0:100], y_visited[0:100], c='blue', s=1)
#         plt.pause(0.005)
#         del x_visited[:100]
#         del y_visited[:100]
#     else:
#         for j in range(len(x_visited)):
#             plt.scatter(x_visited[j], y_visited[j], c='blue', s=1)
#             plt.pause(0.005)
#             x_temp = x_visited[j]
#             y_temp = y_visited[j]
#             if x_visited[j] == goal[0] and y_visited[j] == goal[1]:
#                 break
            
for j in range(len(x_visited)):
            plt.scatter(x_visited[j] , y_visited[j] , c='red' , s=2)
            plt.pause(0.0005)
            if x_visited[j] == goal[0] and y_visited[j] == goal[1] :
                break

plt.title("The shortest Path travelled by the robot")
for i in range(len(path_x_coord)-1):
    ax.quiver(path_x_coord[i], path_y_coord[i], path_x_coord[i+1]-path_x_coord[i], path_y_coord[i+1]-path_y_coord[i], units='xy' ,scale=1)
    plt.pause(0.005)
plt.savefig('Astar algorithm', bbox_inches='tight')
plt.waitforbuttonpress(timeout=-1)
plt.show
# plt.show(block=True)




