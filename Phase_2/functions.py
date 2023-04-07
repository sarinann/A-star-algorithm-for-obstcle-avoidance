import math
from queue import PriorityQueue
import numpy as np
import matplotlib.patches as patch
import matplotlib.pyplot as plt

angle = 15

def actions(ur, ul):
    action_taken = [[0, ul], [ul,0], [ul, ul], [0, ur], [ur, 0], [ur, ur], [ul, ur], [ur, ul]]

    return action_taken

def rounding_value(x, y, angle, theta):
    x = (round(x * 10) / 10)
    y = (round(y * 10) / 10)
    angle = (round(angle / theta) * theta)
    return (x, y, angle)

def map():

    figure, axes = plt.subplots()

    import matplotlib.patches as patches
    axes.set(xlim=(0, 6), ylim=(0, 2))

    circle = plt.Circle((4, 1.1), 0.5, fill='True')

    rectangle_1 = patches.Rectangle((2.5, 0), 0.15, 1.25, color='blue')
    rectangle_2 = patches.Rectangle((1.5, 0.75), 0.15, 1.25, color='blue')

    axes.set_aspect('equal')
    axes.add_artist(circle)
    axes.add_patch(rectangle_1)
    axes.add_patch(rectangle_2)
    plt.show()


def cost(Xi, Yi, Thetai, UL, UR): #function for cost provided
    Thetai = Thetai % 360
    t = 0
    r = 0.038
    L = 0.354
    dt = 0.1
    Xn = Xi
    Yn = Yi
    Thetan = 3.14 * Thetai / 180
    
    # Xi, Yi,Thetai: Input point's coordinates
    # Xs, Ys: Start point coordinates for plot function
    # Xn, Yn, Thetan: End point coordintes

    D = 0
    while t < 1:
        t = t + dt
        Xs = Xn
        Ys = Yn
        Xn += 0.5 * r * (UL + UR) * math.cos(Thetan) * dt
        Yn += 0.5 * r * (UL + UR) * math.sin(Thetan) * dt
        Thetan += (r / L) * (UR - UL) * dt
        D = D + math.sqrt(math.pow((0.5 * r * (UL + UR) * math.cos(Thetan) * dt), 2) + math.pow((0.5 * r * (UL + UR) * math.sin(Thetan) * dt), 2))
    Thetan = 180 * (Thetan) / 3.14
    cost = (*rounding_value(Xn, Yn, Thetan, angle), D, UL, UR)
    return cost

def backtracking(goal, start, generated_path):

    current_node_n = goal

    shortest_path = [goal]

    while current_node_n != start:

        current_node_n = generated_path[current_node_n]

        shortest_path.append(current_node_n)

        return shortest_path[::-1]

def correct_childs(current_node, robot_radius, clearance, ul, ur):

    for action in actions(ul, ur):
        c_x, c_y, c_theta, c_cost, ul, ur = cost(*current_node, *action)

        bloat = robot_radius + clearance

        check_child = (current_node[0], current_node[1], bloat)

        if not if_obstacle(check_child):
                plt.plot([current_node[0], c_x], [current_node[1], c_y], color="red", alpha=0.2)
                plt.pause(0.01)
                yield c_x, c_y, c_theta, c_cost, ul, ur


def Astar_algorithm(start, goal, robot_radius, clearance, ul, ur):
    visited_close_list = {}
    generated_path = {}
    open_list = PriorityQueue()
    initial_cost_to_go = float('inf')

    initial_cost_to_come = 0

    total_cost = initial_cost_to_come + initial_cost_to_go

    open_list.put((start, total_cost, initial_cost_to_come))

    while not open_list.empty():
        current_node_entire = open_list.get()

        current_node = current_node_entire[0]
        cost_to_go = current_node_entire[1]
        cost_to_come = current_node_entire[2]

        visited_close_list[(current_node[0], current_node[1])] = 1

        distance_to_goal = np.sqrt(
            (goal[0] - current_node[3][0])**2 + (goal[1] - current_node[3][1])**2)
        
        if distance_to_goal <= 1.5:
            print("Goal has been reached!!!!")
            print("_____________________________________________________________________________")

            goal = current_node

            generated_path = backtracking(goal, start, generated_path)

            return generated_path
        
        children = set(correct_childs(current_node, robot_radius, clearance, ul, ur))

        for modi_x, modi_y, modi_theta ,modi_cost, ul, ur in children:
            distance_to_g = math.dist((modi_x, modi_y), (goal[0], goal[1]))

            if visited_close_list.get((modi_x, modi_y)) == 1:
                continue
            
            updated_cost = cost_to_come + modi_cost

            modi_total_cost = updated_cost + distance_to_g

            open_list.put(((modi_x, modi_y, modi_theta), modi_total_cost, updated_cost))

            generated_path[(modi_x, modi_y, modi_theta)] = current_node

def show_path_travelled(path):  #function to show the trajectory

    start_node = path[0]
    goal_node = path[-1]
    plt.plot(start_node [0], start_node[1], "Dg")
    plt.plot(goal_node[0], goal_node[1], "Dg")

    for i, (x, y, theta) in enumerate(path[:-1]):
        n_x, n_y, theta = path[i+1]
        plt.plot([x, n_x], [y, n_y], color="black")

    plt.show()
    plt.pause(5)
    plt.close('all')








    


