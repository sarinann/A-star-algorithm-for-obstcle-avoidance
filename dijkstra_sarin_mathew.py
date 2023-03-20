# GitHub Link: https://github.com/sarinann/proj2_sarin_mathew.git

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
def ActionMoveUp(node):
    modified_node = copy.deepcopy(node)
    cost = modified_node[0]+1
    parent_node = modified_node[2]
    x_coord = modified_node[2][0]   
    y_coord = modified_node[2][1]+1
    modified_node = (x_coord, y_coord)
    passed_node = dijkstra_node_create(cost, parent_node, modified_node)
    return passed_node

def ActionMoveDown(node):
    modified_node = copy.deepcopy(node)
    cost = modified_node[0]+1
    parent_node = modified_node[2]
    x_coord = modified_node[2][0]   
    y_coord =modified_node[2][1]-1
    modified_node = (x_coord, y_coord)
    passed_node = dijkstra_node_create(cost, parent_node, modified_node)
    return passed_node

def ActionMoveLeft(node):
    modified_node = copy.deepcopy(node)
    cost = modified_node[0]+1
    parent_node = modified_node[2]
    x_coord = modified_node[2][0]-1
    y_coord =modified_node[2][1]
    modified_node = (x_coord, y_coord)
    passed_node = dijkstra_node_create(cost, parent_node, modified_node)
    return passed_node

def ActionMoveRight(node):
    modified_node = copy.deepcopy(node)
    cost = modified_node[0]+1
    parent_node = modified_node[2]
    x_coord = modified_node[2][0]+1   
    y_coord =modified_node[2][1]
    modified_node = (x_coord, y_coord)
    passed_node = dijkstra_node_create(cost, parent_node, modified_node)
    return passed_node
    
def ActionMoveUp_left(node):
    modified_node = copy.deepcopy(node)
    cost = modified_node[0]+1.4
    parent_node = modified_node[2]
    x_coord = modified_node[2][0]-1   
    y_coord =modified_node[2][1]+1
    modified_node = (x_coord, y_coord)
    passed_node = dijkstra_node_create(cost, parent_node, modified_node)
    return passed_node

def ActionMoveUp_right(node):
    modified_node = copy.deepcopy(node)
    cost = modified_node[0]+1.4
    parent_node = modified_node[2]
    x_coord = modified_node[2][0]+1   
    y_coord =modified_node[2][1]+1
    modified_node = (x_coord, y_coord)
    passed_node = dijkstra_node_create(cost, parent_node, modified_node)
    return passed_node

def ActionMoveDown_left(node):
    modified_node = copy.deepcopy(node)
    cost = modified_node[0]+1.4
    parent_node = modified_node[2]
    x_coord = modified_node[2][0]-1   
    y_coord =modified_node[2][1]-1
    modified_node = (x_coord, y_coord)
    passed_node = dijkstra_node_create(cost, parent_node, modified_node)
    return passed_node

def ActionMoveDown_right(node):
    modified_node = copy.deepcopy(node)
    cost = modified_node[0]+1.4
    parent_node = modified_node[2]
    x_coord = modified_node[2][0]+1   
    y_coord =modified_node[2][1]-1
    modified_node = (x_coord, y_coord)
    passed_node = dijkstra_node_create(cost, parent_node, modified_node)
    return passed_node
    
def dijkstra_algo(start, goal):
    #Initializing the start node
    start_node = dijkstra_node_create(0, None, start)
    #Intializing the queue
    open_queue = PriorityQueue()
    open_queue.put(start_node)
    all_x_visited =[]
    all_y_visited =[]

    #initialize the dictionary to hold the path cost from start to every node
    cost_from_start = {start: 0}

    #initialize the dictionary to hold the parent node for each visited node
    visited_closed_dict = {}

    while not open_queue.empty():
        #Remove and return the lowest-cost node
        present_node = open_queue.get()
        
        if present_node[2] in visited_closed_dict: 
            continue

        all_x_visited.append(present_node[2][0]) 
        all_y_visited.append(present_node[2][1])

        visited_closed_dict[present_node[2]] = (present_node[1]) 
               
        #Check if we have reached the goal coordinates
        if present_node[2] == goal:
            #return the path by backtracking from goal to start
            generated_path = []
            current_pose = present_node[2] 
            while current_pose is not None:
                generated_path.append(current_pose)
                current_pose = visited_closed_dict[current_pose]
            #Reverse the path and return
            return generated_path[::-1], all_x_visited, all_y_visited
        
        #If the goal is not reached, expand the current node
        actions = [ActionMoveUp, ActionMoveDown, ActionMoveLeft, ActionMoveRight, ActionMoveUp_left, ActionMoveUp_right, ActionMoveDown_left, ActionMoveDown_right]
        for action in actions:
            #Get the modified node and its cost to move from current node
            new_node = action(present_node)
            if Obstacle_space(new_node[2][0], new_node[2][1]) == False:
                cost2come = new_node[0]            
                #If the node is not in the cost dictionary, add it with infinite cost
                if new_node[2] not in visited_closed_dict:
                    for i in range(0,(open_queue.qsize())):
                        if open_queue.queue[i][2] == new_node[2] and open_queue.queue[i][0] > cost2come:
                            open_queue.queue[i][1] == new_node[1]
                    open_queue.put(new_node)
                    cost_from_start[new_node[2]] = float('inf')
                
    #If the goal is not reached and the queue is empty, there is no path to the goal
    return None

start_point_x = input("Enter the x-coordinate of the start point \n")
start_point_y = input("Enter the y-coordinate of the start point \n")
start= (int(start_point_x), int(start_point_y))
while Obstacle_space(start[0], start[1]):
    print("These coordinates lie inside the obstacle space. Please enter new values\n")
    start_point_x = input("Enter the x-coordinate of the start point \n")
    start_point_y = input("Enter the y-coordinate of the start point \n")
    start= (int(start_point_x), int(start_point_y))

goal_point_x = input("Enter the x-coordinate of the goal point \n")
goal_point_y = input("Enter the y-coordinate of the goal point \n")
goal= (int(goal_point_x), int(goal_point_y))
while Obstacle_space(goal[0], goal[1]):
    print("These coordinates lie inside the obstacle space. Please enter new values\n")
    goal_point_x = input("Enter the x-coordinate of the goal point \n")
    goal_point_y = input("Enter the y-coordinate of the goal point \n")
    goal= (int(goal_point_x), int(goal_point_y))

generated_path, x_visited, y_visited = dijkstra_algo(start, goal)
end_time = time.time()
print("Reached Goal")
# print("The generated path is")
# print(generated_path)
# print("The x_coordinates of the visited nodes")
# print(x_visited)
# print("The y_coordinates of the visited nodes")
# print(y_visited)
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

x_temp = 0
y_temp = 0

for i in range(len(x_visited)) :
    if x_temp == goal[0] and y_temp == goal[1] :
        break
    if len(x_visited)>100:
        plt.scatter(x_visited[0:100] , y_visited[0:100] , c='blue' , s=1)
        plt.pause(0.0005)
        del x_visited[:100]
        del y_visited[:100]
    else :
        for j in range(len(x_visited)):
            plt.scatter(x_visited[j] , y_visited[j] , c='blue' , s=1)
            plt.pause(0.0005)
            x_temp = x_visited[j]
            y_temp = y_visited[j]
            if x_visited[j] == goal[0] and y_visited[j] == goal[1] :
                break

# for j in range(len(x_visited)):
#             plt.scatter(x_visited[j] , y_visited[j] , c='red' , s=1)
#             plt.pause(0.0005)
#             if x_visited[j] == goal[0] and y_visited[j] == goal[1] :
#                 break

plt.title("The shortest Path travelled by the point robot")
for i in range(len(path_x_coord)):
    plt.scatter(path_x_coord[i] , path_y_coord[i] , c='yellow' , s=2, marker='D')
    plt.pause(0.0005)
plt.waitforbuttonpress(timeout=-1)
plt.show
