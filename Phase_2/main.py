#importing the files Obstacles and Functions
from obs import *
from functions import *
import time

#Defining clearance and robot radius
clearance = 5
# clearance = int(input("Enter the clearance to be maintained around the obstacle"))
# robot_radius = int(input("Enter the radius of the robot \n"))
robot_radius = 105

#converting the dimensions to metres
robot_radius = robot_radius/1000
clearance = clearance/1000
#Total amount the boundary needs to be bloated by taking into account robot radius and clearance.
total_bloat = robot_radius + clearance

start_time = time.time()
print("Hi!! \n")
print("______________________________________________________________________")
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

ul = int(input("Enter the velocity of left wheel \n"))
print("______________________________________________________________________")

ur = int(input("Enter the velocity of right wheel \n"))
print("______________________________________________________________________")

generated_path = Astar_algorithm(start, goal, robot_radius, clearance, ul, ur)
end_time = time.time()
print("Printing generated path \n")
print(generated_path)

print(f'Time taken to solve using the A* algorithm: {end_time - start_time} \n')
print("_____________________________________________________________________________")