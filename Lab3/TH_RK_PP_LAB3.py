import rospy, tf, numpy, math, sys
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Pose, Point
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import GridCells

from Queue import PriorityQueue
from math import sqrt
 
adjustCell = 0.5


###############MAP CALLBACK###############
##########################################
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
##########################################
##########################################




#Publish cells given a publisher
def publishCells(info, pub):
    print "publishing"

    # resolution and offset of the map
    cells = GridCells()
    cells.header.frame_id = 'map'
    cells.cell_width = resolution 
    cells.cell_height = resolution
    
    for node in info:
        point = nodeToPoint(node)
        cells.cells.append(point)
    pub.publish(cells)

def convertToGridCell(x, y):
	xCell = int(((x - offsetX) - adjustCell * resolution) / resolution)
	yCell = int(((y - offsetY) - adjustCell * resolution) / resolution)
    return xCell, yCell 

#Convert a node into its position on a global axis
def nodeToPoint(node):
    point = Point()
    point.x = (node.x * resolution) + (adjustCell * resolution) + offsetX
    point.y = (node.y * resolution) + (adjustCell * resolution) + offsetY
    return point

def convertInitNode(pose):
    global init_x
    global init_y
   
    init_x, init_y = convertToGridCell(pose.pose.pose.position.x , pose.pose.pose.position.y)
    

def convertFinalNode(pose):
    global final_x_cord
    global final_y_cordfinal_
    final_x_cord, final_y_cord = convertToGridCell(pose.pose.position.x, pose.pose.position.y)
    endNode = Node(None, final_x_cord, final_y_cord, 0)
    runAStar(Node(None, init_x_cord, init_y_cord, 1), endNode)
    

#node class 
#some simple attributes for a node

class Node():
 
    def __init__(self, parent, x, y, cost):
        self.parent = parent
        self.x = x
        self.y = y
        self.cost = cost
 
#ASTAR FUNCTIONS#
#################
def runAStar(startNode, goalNode):
    global closedSet
    global openSet
    closedSet = []
    openSet = []
    path = []
    wayPoints = []
    aStar(startNode, goalNode, path, closedSet, openSet, wayPoints)
    return path, wayPoints
 
# aStar algorithm
def aStar(startNode, goalNode, path, closedSet, openSet, wayPoints):
    global pub_closed
    openSet.append(startNode)
    
    #While there are elements in the openset
    while len(openSet) != 0:
        openSet = list(sorted(openSet, key=lambda x: totalCost(x, goalNode)))
        currentNode = openSet[0]
        openSet.remove(currentNode)
        closedSet.append(currentNode)
 
        #if the node we are at is the goal
        if currentNode == goalNode:
            path = pathTrace(currentNode, path, wayPoints)
            publishCells(path, pub_path)
            break
 
        # build a set of the neighbors of the current grid cell
        children = getNeighbors(currentNode)
 
        # for each of the next nodes, either put it into the open set
        # using its cost as a priority or ignore it if it is in the closed set
        for child in children:
            if child not in closedSet and child not in openSet:              
                openSet.append(child)

            elif child in closedSet:
                childIndex = closedSet.index(child)
                if closedSet[childIndex].cost > child.cost:
                	openSet.append[child]                      
                    
            elif child in openSet:
                childIndex = openSet.index(child)
                if openSet[childIndex].cost > child.cost:
                    openSet.append(child)
                    
            else:
                print "Ignoring", child, " (it is further to get here)"
        publishCells(closedSet, pub_closed)
        publishCells(openSet, pub_open)

        # dummy = raw_input("Press Enter to continue...")
 
# can add "turning cost"
def totalCost(node, goalNode):
    return node.cost + getHeuristic(node, goalNode)
 
# this functions returns the heuristic value of the node
# at the moment it returns zero, which makes the algorithm
# behave like greeyDir search
def getHeuristic(node, goalNode):
    return sqrt(pow(goalNode.x - node.x, 2) + pow(goalNode.y - node.y, 2))
 
# pathtrace shows the path found by the algorithm
def pathTrace(node, path, wayPoints):
    path.append(node)
    publishCells(path, pub_path)
    publishCells(wayPoints, way_pub)
    rospy.sleep(0.05)
    while node.parent is not None:
    	if(node.parent.parent.x != node.x and node.parent.parent.y != node.y):
    		wayPoints.append(node.parent)
        pathTrace(node.parent, path,wayPoints)
    	

    return path, wayPoints
   
 
# returns the neighbors of this node
# will need to add check for free/obstacle
def getNeighbors(node):
    adjacent = []
    for xDir in [-1,0, 1]:
        for yDir in [-1,0,1]:
          
          	#check if the node is at the edge
            if node.x <= -width/2 and xDir == -1:
                pass
            if node.x >= width/2 and xDir == 1:
                pass
            if node.y <= -height/2 and yDir ==-1:
                pass
            if node.y >= height/2 and yDir == 1:
                pass
            
            if not (xDir == 0 and yDir == 0):
            	#if the node isn't occupied
                if mapData[int((node.y+yDir) * width + (node.x+xDir))] < 50:
                	#if only 1 of them is 1, this will give us 4 neighbors, not 8
                    if not ( abs(xDir) == 1 and abs(yDir) == 1):
                        neighbors.append(Node(node, node.x + xDir, node.y +yDir, node.cost+1))
                  
    return adjacent
##########################################################

def run():
    global pub_open
    global pub_closed
    global pub_path
    global way_pub

    global mapData
    global width
    global height
    global mapgrid
    global resolution
    global offsetX
    global offsetY
       
    rospy.init_node('lab3')
   
   
    pub_open = rospy.Publisher("/openCells", GridCells, queue_size=1) 
    pub_closed = rospy.Publisher("/closedCells", GridCells, queue_size=1) 
    pub_path = rospy.Publisher("/TomIsGreat", GridCells, queue_size=1) 
    way_pub = rospy.Publisher("/ParmSucks", GridCells, queue_size=1) 

    sub = rospy.Subscriber("/map", OccupancyGrid, mapCallBack)
    sub_initPose = rospy.Subscriber('/initialpose1', PoseWithCovarianceStamped, convertInitNode, queue_size=10)   #initail pose 1 
    sub_finalPose = rospy.Subscriber('/move_base_simple/goal1', PoseStamped, convertFinalNode, queue_size=10) #goal 1 - need that to avoid Rviz running built in stuff
    rospy.sleep(10)



    while (1 and not rospy.is_shutdown()):
        rospy.sleep(2)  
     

if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass


