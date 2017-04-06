#!/usr/bin/env python

import rospy, tf, numpy, math, heapq
from nav_msgs.msg import GridCells
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Point, Pose, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, OccupancyGrid
#from kobuki_msgs.msg import BumperEvent

class aTom:    #tom is a node
    def __init__ (self,x,y,h,g,f,prob, index): # x= xpos, y= ypos, h= heuristic, g= g(n), f= g+h, prob= prob of being a wall
        self.x = x
        self.y = y
        self.h = h
        self.g = g
        self.f = 0
        self.index=index
        self.prob = prob
        self.neighbors = list()
        self.camefrom = None

def makeGrid (cells):
    H = cells.info.height
    W = cells.info.width

    index = 0


    global grid
    global row
    global lib

    grid = list()
    row = list()
    lib = list()

    for i in range(1,W):
        for j in range(1,H):
            newTom = aTom (i, j, 0, 0, 0, mapData[index],0)
            row.append(index)
            lib.append(newTom)
            index += 1
        grid.append(row)


    for i in range (1,W-1):
        for j in range (1,H-1):
            if (i > 0 and lib[grid[j][i-1]].prob < 90):
                lib[grid[i][j].neighbors.append(grid[i-1][j])
            if (i < W-1 and lib[grid[j][i+1]].prob < 90):
                lib[grid[i][j].neighbors.append(grid[i+1][j])
            if (j > 0 and lib[grid[j-1][i]].prob < 90):
                lib[grid[i][j].neighbors.append(grid[i][j-1])
            if (j < H-1 and lib[grid[j+1][i]].prob < 90):
                lib[grid[i][j].neighbors.append(grid[i][j+1])


def posePosToNode(pose)
    gridX = int ((pose.position.x - offsetX -(0.5*resolution))/resolution 
    gridY = int ((pose.position.y - offsetY -(0.5*resolution))/resolution 
    return grid[gridX][gridY]

def heuristic(index):
    lib[index].h = math.sqrt((math.pow(lib[goalNode].x-lib[index].x),2) + (math.pow(lib[goalNode].y-lib[index].y),2))

def sets():

    global explored 
    global unExplored

    # ecell = explored cell
    ecells = GridCells()
    ecells.header.frame_id = 'map'
    ecells.cell_width = resolution 
    ecells.cell_height = resolution

    for i in explored:
        tom = lib[i]
        point = Point()
        point.x = (tom.x * resolution) + offsetX + (0.5 * resolution)
        point.y = (tom.y * resolution) + offsetY + (0.5 * resolution)
        point.z = 0
        ecells.cells.append(point)
    ePub.publish(ecells)
        
    # ucell = unExplored cell
    ucells = GridCells()
    ucells.header.frame_id = 'map'
    ucells.cell_width = resolution 
    ucells.cell_height = resolution

    for i in unExplored:
        tom = lib[i]
        point = Point()
        point.x = (tom.x * resolution) + offsetX + (0.5 * resolution)
        point.y = (tom.y * resolution) + offsetY + (0.5 * resolution)
        point.z = 0
        ucells.cells.append(point)
    uPub.publish(ucells)


#THE FUNCTIONS FOR A*#

def getLowestPriority(aList)
    lowestPriority = lib[aList[0]]
    for i in aList:
        if (lib[i].f < lowestPriority.f):
            lowestPriority.f = lib[i].f
    return lowestPriority.index

def makePath(start, end, aMap):
    copyMapForAStar(aMap)
    global startNode,endNode,endPose
    startNode = posePosToNode(start.pose.pose)
    endNode = posePosToNode(end.pose.pose)
    endPose = end

    #Publishing globals
    global uPub, ePub, optimalPath
    uPub = rospy.Publisher("/AStar/unExploredNodes",GridCells, queue_size = 1)
    ePub = rospy.Publisher("/AStar/exploredNodes", GridCells, queue_size = 1)
    optimalPath = rospy.Publisher("/AStar/optimalPathNodes", GridCells, queue_size=1)

    global unExplored,explored
    unExplored = list()
    explored = list()
    explored.append(start)


    while unExplored:
        global currentNode
        currentNode = getLowestPriority(unExplored)

        sets()

        unExplored.remove(currentNode)
        explored.append(currentNode)

        if (currentNode == endNode):
            sets()
            return makeOptimalPath(currentNode, startNode)
        
        for adjacent in lib[currentNode].neighbors:
            if (adjacent not in explored):
                heuristic(adjacent)
                lib[adjacent].g = lib[currentNode].g + 1
                lib[adjacent].f = lib[adjacent].g + lib[adjacent].h
                lib[adjacent].camefrom = currentNode
                explored.append = adjacent

        for rayyan in explored:
            if rayyan in unExplored:
                explored.remove(rayyan)
    

def copyMapForAStar(data):
    global mapData
    global width
    global height
    global mapgrid
    global resolution
    global offsetX
    global offsetY

    mapgrid = data
    resolution = data.info.resolution
    mapData = data.data
    width = data.info.width
    height = data.info.height
    offsetX = data.info.origin.position.x
    offsetY = data.info.origin.position.y
    makeGrid(mapData)

# reads in global map
def mapCallBack(data):
    global mapData
    global width
    global height
    global mapgrid
    global resolution
    global offsetX
    global offsetY

    mapgrid = data
    resolution = data.info.resolution
    mapData = data.data
    width = data.info.width
    height = data.info.height
    offsetX = data.info.origin.position.x
    offsetY = data.info.origin.position.y

    #print data.info


def publishCells(grid):
    global pub
    print "publishing"

    # resolution and offset of the map
    k=0
    cells = GridCells()
    cells.header.frame_id = 'map'
    cells.cell_width = resolution 
    cells.cell_height = resolution

    
    for i in range(1,height): #height should be set to hieght of grid
        k=k+1
        for j in range(1,width): #width should be set to width of grid
            k=k+1
            #print k # used for debugging
            if (grid[k] == 100):
                point=Point()
                point.x=(j*resolution)+offsetX + (1.5 * resolution) # added secondary offset 
                point.y=(i*resolution)+offsetY - (.5 * resolution) # added secondary offset ... Magic ?
                point.z=0
                cells.cells.append(point)
    pub.publish(cells)           

def neighbors(cell):
    cell.point.x
    cell.point.y

#Main handler of the project
def run():
    global pub
    rospy.init_node('ppatias_lab3')
    sub = rospy.Subscriber("/map", OccupancyGrid, mapCallBack)
    pub = rospy.Publisher("/ourGrid", GridCells, queue_size=1)  
    #pubpath = rospy.Publisher("/path", GridCells, queue_size=1) # you can use other types if desired
    #pubway = rospy.Publisher("/waypoints", GridCells, queue_size=1)
    #goal_sub = rospy.Subscriber('move_base_simple/goal', PoseStamped, readGoal, queue_size=1) #change topic for best results
    #goal_sub = rospy.Subscriber('initialpose', PoseWithCovarianceStamped, readStart, queue_size=1) #change topic for best results

    # wait a second for publisher, subscribers, and TF
    rospy.sleep(1)



    while (1 and not rospy.is_shutdown()):
        publishCells(mapData) #publishing map data every 2 seconds
        rospy.sleep(2)  
        print("Complete")
    


if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass




