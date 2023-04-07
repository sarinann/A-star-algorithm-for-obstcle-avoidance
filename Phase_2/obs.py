import numpy as np
import matplotlib.pyplot as plt
# import cv2 as cv

#Defining the outer boundaries
def wall(input):
    x = input[0]
    y = input[1]
    total_bloat = input[2]
    if (x - total_bloat <= 0) or (x + total_bloat >= 6) or (y - total_bloat <= 0) or (y + total_bloat >= 2):
        return True
    else:
        return False
    
#Defining obstacles
def circle(input):
    x = input[0]
    y = input[1]
    total_bloat = input[2]
    if  (((x-4)**2)+((y-1.1)**2)-((1+total_bloat)**2)) <= 0:
        return True
    else:
        return False

def rectangle_1(input): #obstacle 4
    x = input[0]
    y = input[1]
    total_bloat = input[2]
    if x-(1.5-total_bloat)>= 0 and x-(1.65+total_bloat) <= 0 and y-(0.75-total_bloat) >= 0 and y-(2+total_bloat) <= 0:
        return True
    else:
        return False

def rectangle_2(input): #obstacle 5
    x = input[0]
    y = input[1]
    total_bloat = input[2]
    if x-(2.5-total_bloat) >= 0 and x-(2.65+total_bloat) <= 0 and y-(0-total_bloat) >= 0 and y-(1.25+total_bloat) <= 0:
        return True
    else:
        return False

def if_obstacle(coordinates): #checking all obstacles + boundaries
    if (wall(coordinates) or circle(coordinates) or rectangle_1(coordinates) or rectangle_2(coordinates)) == True:
        return True
    else:
        return False

#Creating map
def final_map(total_bloat):
    total_bloat  *= 100
    #Background
    # Geometrical definition of the obstacle space
    obstaclespace = np.zeros(shape=(int(2001), int(6001))) 
    # Defining the boundary
    boundary_x = []
    boundary_y = []

    for i in range(6001):
        boundary_x.append(i)
        boundary_y.append(0)
        obstaclespace[0][i] = -1

        boundary_x.append(i)
        boundary_y.append(6000)
        obstaclespace[2000][i] = -1

    for i in range(2001):
        boundary_x.append(0)
        boundary_y.append(i)
        obstaclespace[i][0] = -1

        boundary_x.append(1000)
        boundary_y.append(i)
        obstaclespace[i][6000] = -1
    plt.scatter(boundary_x, boundary_y, color='g')

# circle
    circle = []
    for x in range(6001):
        for y in range(2001):
            if (((x-4000)**2)+((y-1100)**2)-((500+total_bloat)**2)) <= 0:
                circle.append((x, y))

    circle_x = [x[0] for x in circle]
    circle_y = [x[1] for x in circle]
    plt.scatter(circle_x, circle_y, color='b')
    for i in circle:
        obstaclespace[i[1]][i[0]] = -1


# rectangle_1
    sideA = []
    sideB = []
    sideC = []
    sideD = []
    
    for x in range(6001):
        for y in range(2001):
            if x-(1.5*1000-total_bloat) >= 0:
                sideA.append((x, y))
            if x-(1.65*1000+total_bloat) <= 0:
                sideB.append((x, y))
            if y-(0.75*1000-total_bloat) >= 0:
                sideC.append((x, y))
            if y-(2*1000+total_bloat) <= 0:
                sideD.append((x, y))
           
    rectangle1side = list(set(sideA) & set(sideB) & set(sideC) & set(sideD))
    for i in rectangle1side:
        obstaclespace[i[1]][i[0]] = -1
    x_rectangle1 = [x[0] for x in rectangle1side]
    y_rectangle1 = [x[1] for x in rectangle1side]
    plt.scatter(x_rectangle1, y_rectangle1, color='b')


# rectangle_2
    sideE = []
    sideF = []
    sideG = []
    sideH = []

    for x in range(1001):
        for y in range(1001):
            if x-(2.5*1000-total_bloat) >= 0:
                sideE.append((x, y))
            if x-(2.65*1000+total_bloat) <= 0:
                sideF.append((x, y))
            if y-(0*1000-total_bloat) >= 0:
                sideG.append((x, y))
            if y-(1.25*1000+total_bloat) <= 0:
                sideH.append((x, y))
           
    rectangle2side = list(set(sideE) & set(sideF) & set(sideG) & set(sideH))
    for i in rectangle2side:
        obstaclespace[i[1]][i[0]] = -1
    x_rectangle2 = [x[0] for x in rectangle2side]
    y_rectangle2 = [x[1] for x in rectangle2side]
    plt.scatter(x_rectangle2, y_rectangle2, color='b')

    obstacle_t = obstaclespace.T
    obs = []
    for i in range(6001):
        # for j in range(2001):
            obs.append(obstacle_t[i])
    
    plt.show()
    # plt.savefig("Map.png")
    return obs, boundary_x, boundary_y