import rospy, tf, numpy, math, sys

from std_msgs.msg import String
from geometry_msgs.msg import Twist, Pose, Point
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import GridCells

from Queue import PriorityQueue
from math import *
from astar import *

 
xAdjust = 0.5
yAdjust = 0.5
def mapCallBack(data):
    global mapData
    global width
    global height
    global mapgrid
    global resolution
    global offsetX
    global offsetY

    offsetX = data.info.origin.position.x
    offsetY = data.info.origin.position.y
    mapgrid = data
    resolution = data.info.resolution
    mapData = data.data
    width = data.info.width
    height = data.info.height
    
  
def readLocalMap(data):
    global local_mapData
    global local_width
    global local_height
    global local_mapgrid
    global local_resolution
    global local_offsetX
    global local_offsetY

    local_offsetX = data.info.origin.position.x
    local_offsetY = data.info.origin.position.y
    local_mapgrid = data
    local_resolution = data.info.resolution
    local_mapData = data.data
    local_width = data.info.width
    local_height = data.info.height
    

def buildPoseStamped(node):
    msg = PoseStamped()
    msg.pose.position = nodeToPoint(node)
    return msg

def publishCells(info, publisher):
    a=0
    cells = GridCells()
    cells.header.frame_id = 'map'
    cells.cell_width = resolution 
    cells.cell_height = resolution
    
    for node in info:
        point = nodeToPoint(node)
        cells.cells.append(point)
	
	publisher.publish(cells)

def convertToGridCell(xClick, yClick):
    return int(((xClick - offsetX) - xAdjust * resolution) / resolution), int(((yClick - offsetY) - yAdjust * resolution) / resolution)

def nodeToPoint(node):
    point = Point()
    point.x = (node.x * resolution) + (xAdjust * resolution) + offsetX
    point.y = (node.y * resolution) + (yAdjust * resolution) + offsetY
    return point

def convertInitNode(pose):
    global initialX
    global initialY
    initialX, inititialY = convertToGridCell(pose.pose.pose.position.x , pose.pose.pose.position.y)
    
def convertFinalNode(pose):
    global finalX
    global finalY
    finalX, finalY = convertToGridCell(pose.pose.position.x, pose.pose.position.y)
    start = Node.makeNode(initialX, initialY)
    goal = Node.makeNode(finalX, finalY)
    global openSet
    global costSoFar
   
    waypoints = []
    while start != goal:
        openSet = PriorityQueue()
        costSoFar = {}
        graph = Graph(width, height, mapData)
        path, waypoints = runNavByWaypoint(start, goal, graph, openSet, costSoFar, publishAll)
        
      
        publishCells(path, pub_path)
        publishCells(waypoints, pub_waypoints)
      
        
       next = waypoints[-1]
        
        pub_myWaypoint.publish(buildPoseStamped(next))
        start = next

  

def isTurn(before, now, after):
    if before.x - after.x != 0 and before.x - now.x !=0:
        slopeAB = float(before.y - after.y) / (before.x - after.x)
        slopeBN= float(before.y - now.y) / (before.x - now.x)
        sameLine = abs(slopeAB-slopeBN) > 0.000001
        return sameLine
    else :
        return now.x - after.x != 0

def runNavByWaypoint(start, goal, graph, openSet, costSoFar, publishAll = None):
    path = aStar(start, goal, graph, openSet, costSoFar, publishAll)
    print "Path length:", len(path)
    waypoints = []
    for i in range(1, len(path)-1):
        if isTurn(path[i-1], path[i], path[i+1]) or atEdgeOfVision(path[i]) and path[i] != start:
            waypoints.append(path[i])
    waypoints.insert(0, goal)
    return path, waypoints

def atEdgeOfVision(node):
    point = nodeToPoint(node)
    yes = point.x < local_offsetX - local_width/2 or point.x < local_offsetX + local_width/2 or point.y  < local_offestY - local_height/2 or point.y < local_offestY + local_height/2
    if yes:
        print node, "is past the edge of the current vision"
    return yes

def publishAll(openSet, costSoFar):
    closed = []
 
    publishCells(costSoFar, pub_closed)
    publishCells(openSet, pub_open)

def run():
    global pub_open
    global pub_closed
    global pub_path
    global pub_waypoints
    global pub_myWaypoint
    global pub_initPose

    global mapData
    global localMap
    global width
    global height
    global mapgrid
    global resolution
    global offsetX
    global offsetY


    
    rospy.init_node('Lab 4')
    
    pub_open = rospy.Publisher("/openCells", GridCells, queue_size=1) 
    pub_closed = rospy.Publisher("/closedCells", GridCells, queue_size=1) 
    pub_path = rospy.Publisher("/pathCells", GridCells, queue_size=1) 
    pub_waypoints = rospy.Publisher("/waypointCells", GridCells, queue_size = 1)
    pub_myWaypoint = rospy.Publisher('/mywaypoint',PoseStamped, queue_size = 1)


    sub = rospy.Subscriber("/exp_map", OccupancyGrid, mapCallBack)
    sub_initPose = rospy.Subscriber('/initialpose1', PoseWithCovarianceStamped, convertInitNode, queue_size=10)   #initail pose 1 
    sub_finalPose = rospy.Subscriber('/move_base_simple/goal1', PoseStamped, convertFinalNode, queue_size=10) #goal 1 - need that to avoid Rviz running built in stuff
    sub_exp_map = rospy.Subscriber('/expandMap', OccupancyGrid, readLocalMap, queue_size = 1)
    rospy.sleep(10)



    while (1 and not rospy.is_shutdown()):
        rospy.sleep(2)  


if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass


