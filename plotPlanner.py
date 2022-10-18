# Created by Miles Sigel and Alex Prucka COMP 450

from mpl_toolkits.mplot3d import Axes3D, art3d
from distutils.command.config import config
import matplotlib.patches as patches
import matplotlib.pyplot as plt
from math import sin, cos
import sys

def plotEnvs(envNum):
    fig = plt.figure()
    ax = fig.gca()

    if (envNum == 1):
        plotEnv1Obstacles(ax)
    else:
        plotEnv2Obstacles(ax)

    plt.axis([-3,3,-3,3])
    plt.show()


# plots the first env obstacles
def plotEnv1Obstacles(ax):
    # Drawing the unit square
    delta_x = 0.2
    delta_y = 3.6
    start_x = -1.5
    start_y = -3
    add_patch(ax, start_x=start_x, start_y=start_y, delta_x=delta_x, delta_y=delta_y)

    delta_x2 = 3.6
    delta_y2 = 0.2
    start_x2 = 1.5
    start_y2 = -2.5
    add_patch(ax, start_x=start_x2, start_y=start_y2, delta_x=delta_x2, delta_y=delta_y2)
    
# plots the second env obstacles
def plotEnv2Obstacles(ax):
    obstacles = [[-3, -1.5, 3, 0.1], [0, -2, 0.1, 1], [0, 1.5, 0.1, 3], [0, 1, 0.1, 1]]
    for o in obstacles:
        add_patch(ax, o[0], o[1], o[2], o[3])

def add_patch(ax, start_x, start_y, delta_x, delta_y):
    ax.add_patch(patches.Polygon([(start_x, start_y),
                                (start_x + delta_x, start_y),
                                (start_x + delta_x, start_y + delta_y),
                                (start_x, start_y + delta_y)], 
                                fill=True, color='0.20'))


# plots a point given the enviornment that the point was slotted for
def plotPoint(path, enviornment):
    fig = plt.figure()
    ax = fig.gca()

    if (enviornment == 1):
        plotEnv1Obstacles(ax)
    else:
        plotEnv2Obstacles(ax)

    # Plotting the path
    X = [p[0] for p in path]
    Y = [p[1] for p in path]
    ax.plot(X, Y)

    plt.axis([-3,3,-3,3])
    plt.show()

# plots a box env
def plotBox(path, enviornment):
    fig = plt.figure()
    ax = fig.gca()

    if (enviornment == 1):
        plotEnv1Obstacles(ax)
    else:
        plotEnv2Obstacles(ax)

    # extract x and y 
    X = [p[0] for p in path]
    Y = [p[1] for p in path]
    ax.plot(X, Y)

    # this is the box sizes
    boxVert = [[-0.25, -0.25], [0.25, -0.25], [0.25, 0.25], [-0.25, 0.25], [-0.25, -0.25]]

    for p in path:
        x = []
        y = []
        for v in boxVert:
            x.append(v[0] * cos(p[2]) - v[1] * sin(p[2]) + p[0])
            y.append(v[0] * sin(p[2]) + v[1] * cos(p[2]) + p[1])
        ax.plot(x, y, 'k')

    plt.axis([-3,3,-3,3])
    plt.show()

def readPath(filename):
    linesFromFile = [line.rstrip() for line in open(filename) if len(line.rstrip()) > 0]

    if len(linesFromFile) == 0:
        print("the file passed does not contain valid planning")
        sys.exit(1)

    configuration_space = "point" if len(linesFromFile[0].split(' ')) == 2 else "box"
    point_data = [[float(x) for x in line.split(' ')] for line in linesFromFile]

    return configuration_space, point_data

if __name__ == '__main__':
    if len(sys.argv) > 2:
        filename = sys.argv[1]
        enviornment =  int(sys.argv[2])
    else:
        print("There is an error in the program call")
        print("The first arguement should be a path to a .txt file produced by planner ")
        print("The second arguement should be which enviornment the planner was used on (1 or 2)")
        sys.exit(1)

    # way of just printing the env 
    if filename == "env":
        plotEnvs(enviornment)
        sys.exit(0)
    
    configuration_space, data = readPath(filename)

    if configuration_space == "point":
        plotPoint(data, enviornment)
    if configuration_space == "box":
        plotBox(data, enviornment)
