#Git Hub Link: https://github.com/sarinann/A-star-algorithm-for-obstcle-avoidance.git

import matplotlib.pyplot as plt
import matplotlib.patches as patch
from queue import PriorityQueue
import time
import copy
import numpy as np
from math import dist

start_time = time.time()
threshold = 0.5

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

def a_star_node_create(total_cost, cost_to_come, parent, node):
    return (total_cost,cost_to_come, parent, node)
    
start_point_x = input("Enter the x-coordinate of the start point \n")
start_point_y = input("Enter the y-coordinate of the start point \n")
start_point_theta = input("Enter the start orientation \n")
start= (int(start_point_x), int(start_point_y), int(start_point_theta))
while Obstacle_space(start[0], start[1], start[2]):
    print("These coordinates lie inside the obstacle space. Please enter new values\n")
    start_point_x = input("Enter the x-coordinate of the start point \n")
    start_point_y = input("Enter the y-coordinate of the start point \n")
    start_point_theta = input("Enter the start orientation \n")
    start= (int(start_point_x), int(start_point_y), int(start_point_theta))

goal_point_x = input("Enter the x-coordinate of the goal point \n")
goal_point_y = input("Enter the y-coordinate of the goal point \n")
goal_point_orien = input("Enter the goal orientation \n")
goal= (int(goal_point_x), int(goal_point_y), int(goal_point_orien))
while Obstacle_space(goal[0], goal[1], goal[2]):
    print("These coordinates lie inside the obstacle space. Please enter new values\n")
    goal_point_x = input("Enter the x-coordinate of the goal point \n")
    goal_point_y = input("Enter the y-coordinate of the goal point \n")
    goal_point_orien = input("Enter the goal orientation \n")
    goal= (int(goal_point_x), int(goal_point_y), int(goal_point_orien))

step_size = int(input("Enter the stepsize \n"))


goal_x = goal[0]
goal_y = goal[1]
 
def ActionMove_neg60(node):
    modified_node = copy.deepcopy(node)
    cost = modified_node[1]+1
    parent_node = modified_node[3]

    modi_theta = (parent_node[2] - 60) % 360
    modi_x = parent_node[0] + (step_size*np.cos(np.radians(modi_theta)))
    modi_y = parent_node[1] + (step_size*np.sin(np.radians(modi_theta)))
    modi_theta = rounded_value(modi_theta)
    modi_x = rounded_value(modi_x)
    modi_y = rounded_value(modi_y)
    modified_node = (modi_x, modi_y, modi_theta)

    cost_to_go = dist((modi_x, modi_y), (goal_x, goal_y))

    total_cost = cost + cost_to_go

    passed_node = a_star_node_create(total_cost, cost, parent_node, modified_node)
    return passed_node

def ActionMove_neg30(node):
    modified_node = copy.deepcopy(node)
    cost = modified_node[1]+1
    parent_node = modified_node[3]

    modi_theta = (parent_node[2] - 30) % 360
    modi_x = parent_node[0] + (step_size*np.cos(np.radians(modi_theta)))
    modi_y = parent_node[1] + (step_size*np.sin(np.radians(modi_theta)))
    modi_theta = rounded_value(modi_theta)
    modi_x = rounded_value(modi_x)
    modi_y = rounded_value(modi_y)
    modified_node = (modi_x, modi_y, modi_theta)

    cost_to_go = dist((modi_x, modi_y), (goal_x, goal_y))

    total_cost = cost + cost_to_go

    passed_node = a_star_node_create(total_cost, cost, parent_node, modified_node)
    return passed_node

def ActionMove_zero(node):
    modified_node = copy.deepcopy(node)
    cost = modified_node[1]+1
    parent_node = modified_node[3]

    modi_theta = (parent_node[2]) % 360
    modi_x = parent_node[0] + (step_size*np.cos(np.radians(modi_theta)))
    modi_y = parent_node[1] + (step_size*np.sin(np.radians(modi_theta)))
    modi_theta = rounded_value(modi_theta)
    modi_x = rounded_value(modi_x)
    modi_y = rounded_value(modi_y)
    modified_node = (modi_x, modi_y, modi_theta)

    cost_to_go = dist((modi_x, modi_y), (goal_x, goal_y))

    total_cost = cost + cost_to_go

    passed_node = a_star_node_create(total_cost, cost, parent_node, modified_node)
    return passed_node


def ActionMove_pos30(node):
    modified_node = copy.deepcopy(node)
    cost = modified_node[1]+1
    parent_node = modified_node[3]

    modi_theta = (parent_node[2] + 30) %360
    modi_x = parent_node[0] + (step_size*np.cos(np.radians(modi_theta)))
    modi_y = parent_node[1] + (step_size*np.sin(np.radians(modi_theta)))
    modi_theta = rounded_value(modi_theta)
    modi_x = rounded_value(modi_x)
    modi_y = rounded_value(modi_y)
    modified_node = (modi_x, modi_y, modi_theta)

    cost_to_go = dist((modi_x, modi_y), (goal_x, goal_y))

    total_cost = cost + cost_to_go

    passed_node = a_star_node_create(total_cost, cost, parent_node, modified_node)
    return passed_node

def ActionMove_pos60(node):
    modified_node = copy.deepcopy(node)
    cost = modified_node[1]+1
    parent_node = modified_node[3]

    modi_theta = (parent_node[2] + 60) % 360
    modi_x = parent_node[0] + (step_size*np.cos(np.radians(modi_theta)))
    modi_y = parent_node[1] + (step_size*np.sin(np.radians(modi_theta)))
    modi_theta = rounded_value(modi_theta)
    modi_x = rounded_value(modi_x)
    modi_y = rounded_value(modi_y)
    modified_node = (modi_x, modi_y, modi_theta)

    cost_to_go = dist((modi_x, modi_y), (goal_x, goal_y))

    total_cost = cost + cost_to_go

    passed_node = a_star_node_create(total_cost, cost, parent_node, modified_node)
    return passed_node

def rounded_value(input):
    if input % 0.5 !=0:
        input = (np.round(input/threshold))*threshold
    return input

while Obstacle_space(goal[0], goal[1], goal[2]):
    print("These coordinates lie inside the obstacle space. Please enter new values\n")
    goal_point_x = input("Enter the x-coordinate of the goal point \n")
    goal_point_y = input("Enter the y-coordinate of the goal point \n")
    goal_orien = input("Define the final orientation in degrees - Options: -60,-30, 0, 30, 60")
    goal= (int(goal_point_x), int(goal_point_y), float(goal_orien))

plotting = {}

def Astar_algoritm(start, goal):

    initial_cost_to_go = np.sqrt((goal[0]-start[0])**2 + (goal[1]-start[1])**2 )

    initial_cost = initial_cost_to_go + 0

    zeros = np.zeros((int(600/0.5),int(250/0.5),12))

    start_pose = a_star_node_create(initial_cost, 0, None, start)

    open_list = PriorityQueue()
    open_list.put(start_pose)

    all_x_visited =[]
    all_y_visited =[]
    close_list = {}

    while not open_list.empty():

        current_node = open_list.get()

        # modi_current_node = copy.deepcopy(current_node)

        if current_node[3] in close_list:
            continue

        all_x_visited.append(current_node[3][0]) 
        all_y_visited.append(current_node[3][1])

        # visited_closed_dict[present_node[2]] = (present_node[1]) 
        close_list[current_node[3]] = current_node[2]  

        plotting[current_node[3]] = [] 

        theta = current_node[3][2]

        distance_to_goal = np.sqrt((goal[0] - current_node[3][0])**2 + (goal[1] - current_node[3][1])**2)

        if distance_to_goal <= 1.5:
            print("Goal has been reached!!!!")
            generated_path = []
            current_pose = current_node[3] 
            while current_pose is not None:
                generated_path.append(current_pose)
                current_pose = close_list[current_pose]
            #Reverse the path and return
            return generated_path[::-1], all_x_visited, all_y_visited

        
        actions = [ActionMove_neg60, ActionMove_neg30, ActionMove_zero, ActionMove_pos30, ActionMove_pos60]
        for action in actions:
            #Get the modified node and its cost to move from current node
            child_node = action(current_node)     
            new_node = child_node[3]                   
                #If the node is not in the cost dictionary, add it with infinite cost
            if zeros[int(new_node[0]/0.5)][int(new_node[1]/0.5)][int(new_node[2]/30)]==0:
                    zeros[int(new_node[0]/0.5)][int(new_node[1]/0.5)][int(new_node[2]/30)]=1
                    if Obstacle_space(new_node[0], new_node[1], theta) == False:
                        total_cost = child_node[0]
                        if child_node[3] not in close_list:

                            plotting[child_node[2]].append(new_node)
                            for i in range(0,(open_list.qsize())):
                                if open_list.queue[i][3] == child_node[3] and open_list.queue[i][0] > total_cost:
                                    open_list.queue[i][2] = child_node[2]
                            open_list.put(child_node)
                            all_x_visited.append(child_node[3][0]) 
                            all_y_visited.append(child_node[3][1])
                    # cost_from_start[new_node[2]] = float('inf')
    return None
     
generated_path, x_visited, y_visited = Astar_algoritm(start, goal)
# print(y)

end_time = time.time()
# print(generated_path)
print("Reached Goal")

print(f'Time taken to solve using the Dijkstra algorithm: {end_time - start_time}\n')

path_x_coord = [] 
path_y_coord = []
for i in range(len(generated_path)):
    path_x_coord.append(generated_path[i][0])
    path_y_coord.append(generated_path[i][1])

fig, ax = plt.subplots(figsize=(6,2.5))
Triangle = patch.Polygon([(460, 25), (460, 225), (510, 125)], linewidth=1, edgecolor='r', facecolor='r')
Rectangle_Bottom = patch.Rectangle((100, 150), 50, 100, linewidth=1, edgecolor='r', facecolor='r')
Rectangle_Top = patch.Rectangle((100, 0), 50, 100, linewidth=1, edgecolor='r', facecolor='r')
Hexagon = patch.RegularPolygon((300, 125), 6, 75, linewidth=1, edgecolor='r', facecolor='r')
#Plotting the obstacles
ax.add_patch(Triangle)
ax.add_patch(Rectangle_Bottom)
ax.add_patch(Rectangle_Top)
ax.add_patch(Hexagon)

#Plotting the path
plt.xlabel('X-Axis')
plt.ylabel('Y-Axis')
plt.title("Visualizing Explored Nodes through Dijkstra Algorithm")
plt.axis([0 , 600 , 0 ,250])

# print(plotting_dict)
for k, v in plotting.items() :
    for i in v:
        plt.plot([k[0], i[0]],[k[1], i[1]], c='red')

        plt.pause(0.00005)

plt.title("The shortest Path travelled by the point robot")
for i in range(len(path_x_coord)):
    plt.scatter(path_x_coord[i] , path_y_coord[i] , c='blue' , s=20, marker='D')
    plt.pause(0.005)
# plt.waitforbuttonpress(timeout=-1)
plt.show()




