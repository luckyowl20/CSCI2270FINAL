# required libraries
import numpy as np
import matplotlib.pyplot as plt
import random

# settings for the plot generation
from matplotlib.pyplot import rcParams
np.set_printoptions(precision = 3, suppress = True)
rcParams['font.family'] = 'sans-serif'
rcParams['font.sans-serif'] = ['Tahoma']
plt.rcParams['font.size'] = 22

class treeNode():
    def __init__(self, xpos, ypos):
        self.xpos = xpos    # x position of this node in the plane
        self.ypos = ypos    # y position of this node in the plane
        self.children = []  # list of children nodes
        self.parent = None  # parent node reference
    
# class that contains all funcitons required to traverse the tree and add new nodes in the direction of the goal
class RRTAlg():
    def __init__(self, start, goal, numIterations, grid, stepSize):
        self.randomTree = treeNode(start[0], start[1])      # root node of the RRT
        self.goal = treeNode(goal[0], goal[1])              # goal/end node position
        self.nearestNode = None                             # nearest node to this node
        self.iterations = numIterations                     # number of iterations to run this
        self.grid = grid                                    # plane/map where nodes will generate
        self.stepSize = stepSize                            # length of each branch
        self.pathDistance = 0                               # total path to goal distance at this node
        self.nearestDistance = 10000                        # distance to nearest node
        self.numWaypoints = 0                               # number of waypoints
        self.waypoints = []

    # add the selected point to the nearest node and add goal when the goal is reached
    def addChild(self, xpos, ypos):
        if (xpos == self.goal.xpos):
            # add the goal node to the children of the nearest node
            self.nearestNode.children.append(self)
            self.goal.parent = self.nearestNode
        else:
            # add a temp node to the children of the nearest node
            tempNode = treeNode(xpos, ypos)
            tempNode.parent = self.nearestNode
            self.nearestNode.children.append(tempNode) 
    
    # sample a random point within grid limits, x,y corresponds to grid y,x
    def samplePoint(self):
        x = random.randint(1, grid.shape[1]) # select a random x value based on the size of the plane
        y = random.randint(1, grid.shape[0]) # select a random x value based on the size of the plane
        point = np.array([x, y])
        return point

    # steer a distance determined by stepsize from the start to end location, take a step towards the goal/endPos
    def steerToPoint(self, startPos, endPos):
        offset = self.stepSize * self.unitVector(startPos, endPos)
        point = np.array([startPos.xpos + offset[0], startPos.ypos + offset[1]])
        if point[0] >= grid.shape[1]: # x 
            point[0] = grid.shape[1]-1
        if point[1] >= grid.shape[0]: # y
            point[1] = grid.shape[0]-1
        return point
        
    # check if an obstacle lies between the start node and end point of the planned path/step
    def isInObstacle(self, startPos, endPos):
        unit_vector = self.unitVector(startPos, endPos)
        testPoint = np.array([0.0, 0.0])

        for i in range(self.stepSize):
            # move along the supposed path to determine if the line is in the obstacle or not
            testPoint[0] = startPos.xpos + i*unit_vector[0]
            testPoint[1] = startPos.ypos + i*unit_vector[1]

            # check if test point is within obstacle or not
            # rounding is needed to ensure that the correct data type is compared in the condition
            # also grid coordinate order is flipped so y (1) is first then x (0) is second in order
            if (self.grid[round(testPoint[1]), round(testPoint[0])] == 1):
                print(testPoint[0])
                print(testPoint[1])
                return True
        return False

    # find unit vector between 2 points and form a vector from the distance
    def unitVector(self, startPos, endPos):
        vec = np.array([endPos[0] - startPos.xpos, endPos[1] - startPos.ypos])
        unit_vector = vec / np.linalg.norm(vec) # turns the vector into an actual vector format
        return unit_vector

    # find nearest node from a given unconnected point
    def findNearest(self, root, point):
        # return condition of root is NULL (tree is empty)
        if not root:
            return

        # find distance between root and point
        # if this is lower than the nearest distance, set this as the nearest node and update self.nearestDistance
        dist = self.distance(root, point)
        if dist <= self.nearestDistance:
            self.nearestNode = root
            self.nearestDistance = dist

        # recursively call by iterating through children
        for child in root.children:
            self.findNearest(child, point)

    # find euclidian distance between a node and an x, y point
    def distance(self, node, point):
        distance = np.sqrt((node.xpos - point[0])**2 + (node.ypos - point[1])**2) # euclidian distance between node and point
        return distance

    # check if goal has been reached within step size
    def goalFound(self, point):
        if self.distance(self.goal, point) <= self.stepSize:
            return True

    # reset nearst node and nearest distance
    def resetNearestValues(self):
        self.nearestNode = None
        self.nearestDistance = 10000
        pass

    # trace path from goal to start
    def retracePath(self, goal):
        # start at goal end at start node and end recursion when current node reaches start node

        if self.randomTree is not None and goal is not None:
            if goal.xpos == self.randomTree.xpos:
                return

        # add 1 to number of waypoints
        self.numWaypoints += 1
        # insert curentPoint to the waypoints array from the beginning
        currentPoint = np.array([goal.xpos, goal.ypos])
        # add step size to path distance
        # recursive call this until the top condition returns
        self.waypoints.insert(0, currentPoint) # build the waypoints list from bottom up
        self.pathDistance += self.stepSize
        self.retracePath(goal.parent)
        
# setting up the variables for the algorithm and the grid
grid = np.load("C:/Users/Matthew/Desktop/2024/CSCI2270/final_project/map.npy")
start = np.array([100.0, 100.0])
goal = np.array([1500.0, 750.0])
numIterations = 400
stepSize = 50
goalRegion = plt.Circle((goal[0], goal[1]), stepSize, color='b', fill=False)
pause_time = 0.0001

# setting up the plot
fig = plt.figure("RRT Algorithm")
plt.imshow(grid, cmap = 'binary')
plt.plot(start[0], start[1], 'ro')
plt.plot(goal[0], goal[1], 'bo')
ax = fig.gca()
ax.add_patch(goalRegion)
plt.xlabel("X axis")
plt.ylabel("y axis")


# actual algorithm implementation
rrt = RRTAlg(start, goal, numIterations, grid, stepSize)

# algorithm loop
for i in range(rrt.iterations):
    # reset nearest values
    rrt.resetNearestValues()
    print("Iteration:, ", i)

    # find a point to move in the direction of
    point = rrt.samplePoint()
    rrt.findNearest(rrt.randomTree, point)
    new_point = rrt.steerToPoint(rrt.nearestNode, point)

    in_obstacle = rrt.isInObstacle(rrt.nearestNode, point)
    if in_obstacle == False:
        rrt.addChild(new_point[0], new_point[1])
        plt.pause(pause_time)
        plt.plot([rrt.nearestNode.xpos, new_point[0]], [rrt.nearestNode.ypos, new_point[1]], 'go', linestyle='solid') # plot the new path/node on the plot
        
        # if goal is found, append to path
        if (rrt.goalFound(new_point)):
            rrt.addChild(goal[0], goal[1])
            print("Goal found")
            break


# trace back the path returned and add start to waypoints on the plot
rrt.retracePath(rrt.goal)
rrt.waypoints.insert(0, start)
print("Number of waypoints", rrt.numWaypoints)
print("Path distance: ", rrt.pathDistance)
print("Waypoints:", rrt.waypoints)

# plottiny waypoints on graph
print("length of waypoints ", len(rrt.waypoints))
print(rrt.waypoints)
for i in range(len(rrt.waypoints)-1):
    print("trying to plot waypoint: ", i)
    plt.plot([rrt.waypoints[i][0], rrt.waypoints[i+1][0]], [rrt.waypoints[i][1], rrt.waypoints[i+1][1]], 'ro', linestyle='solid')
    plt.pause(pause_time)

plt.show()