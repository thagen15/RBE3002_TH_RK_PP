import rospy, tf, copy, math

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, 
from tf.transformations import euler_from_quaternion

# given a tf transformListener, returns the current pose of the robot
# as a PoseWithCovarianceStamped message
def getPoseWithCovarianceStampedFromOdom(odom_list):
    initPose = PoseWithCovarianceStamped
    odom_list.waitForTransform('odom','base_footprint', rospy.Time(0), rospy.Duration(5.0)) 
    (position, orientation) = odom_list.lookupTransform('odom','base_footprint', rospy.Time(0))
    initPose = PoseWithCovarianceStamped()
    initPose.pose.pose.position.x = position[0] 
    initPose.pose.pose.position.y = position[1]
    initPose.pose.pose.position.z = position[2]
    quat = orientation         #in quaternion
    q = [quat[0], quat[1], quat[2], quat[3]] 
    roll, pitch, yaw = euler_from_quaternion(q) #from quat to euler 

    initPose.pose.pose.orientation.z = yaw    
    
    return initPose

# given an x and y coordinate, returns the x,y grid coordinates of 
# the same point in the globalMap
def convertToGridCoords(xPos, yPos, globalMap):
    resolution = globalMap.info.resolution
    return int(((xPos - globalMap.info.origin.position.x) - 0.5 * resolution) / resolution), int(((yPos - globalMap.info.origin.position.y) - 0.5 * resolution) / resolution)

# given a node and occupancy grid, returns the real-valued point
# that corresponds to the node
def nodeToPoint(node, globalMap):
    resolution = glboalMap.info.resolution
    offsetX = globalMap.info.origin.position.x
    offsetY = globalMap.info.origin.position.y
    point = Point()
    point.x = (node.x * resolution) + (0.5 * resolution) + offsetX
    point.y = (node.y * resolution) + (0.5 * resolution) + offsetY
    return point

# publishes the list of nodes as GridCells to the given
# rospy publisher
def publishCells(listOfNodes, publisher, globalMap):
    # print "publishing", str(publisher)
    resolution = globalMap.info.resolution
    # resolution and offset of the map
    a=0
    cells = GridCells()
    cells.header.frame_id = 'map'
    cells.cell_width = resolution 
    cells.cell_height = resolution
    
    for node in info:
        point = nodeToPoint(node)
        cells.cells.append(point)
    #pub.publish(cells)
    publisher.publish(cells)

# builds a poseStamped message that contains the location of the given node
def buildPoseStamped(node, globalMap):
    msg = PoseStamped()
    orientation = tf.transformations.quaternion_from_euler(0, 0, 0)

    msg.pose.position = nodeToPoint(node, globalMap)
    msg.pose.orientation.x = orientation[0]
    msg.pose.orientation.y = orientation[1]
    msg.pose.orientation.z = orientation[2]
    msg.pose.orientation.w = orientation[3]

    return msg


class Node:
    
    def __init__(self, x, y, g, parent):
        self.x = x
        self.y =y
        self.g = g
        self.h = 0
        self.parent = parent
        self.obstacleProb = 50

    
    def makeNode(x, y, g = 0, parent = None):
        return Node(x, y, g, parent)

    def __eq__(self, other):
        if isinstance(other, self.__class__):
            return abs(other.x - self.x) < 0.00001 and abs(other.y - self.y) < 0.00001
        else:
            return False

    def __str__(self):
        return "(" + str(self.x) + "," + str(self.y) + ") " + str(self.g) + " h = "+ str(self.h) # + " " + str(self.parent) + ";"

    def __hash__(self):
        return 137*self.x + 149 * self.y


# the Graph class is build from an OccupancyGrid message, 
# and is used to populate a graph of Node objects from the map
# data contained in the occupancy grid.
class Graph:

    def __init__(self, width, height, mapData):
        self.width = width
        self.height = height
        self.graph = {}
        self.map = mapData
        self.cellStates = mapData.data
        # self.constructGraph(width, height)

    def constructGraph(self, width, height):
        neighborsOf = {}
        for x in range(0, width):
            for y in range(0, height):
                newNode = Node.makeNode(x,y)
                neighborsOf[newNode] = self.getNeighbors(newNode)

        return neighborsOf

    # returns the real-valued distance between nodeA and nodeB
    def distance(self, nodeA, nodeB):
        return abs(math.sqrt(pow(nodeA.x - nodeB.x, 2) + pow(nodeA.y - nodeB.y,2)))

    # returns a list of nodes that are adjacent to the given node (aNode)
    def getNeighbors(self, aNode):
        if aNode in self.graph:
            return self.graph[aNode]

        neighbors = []
        # iterate through the eight nodes surrounding aNode
        for dx in [-1,0,1]:
            x = aNode.x + dx
            for dy in [-1,0,1]:
                y = aNode.y + dy

                if not (x == aNode.x and y == aNode.y) and self.isValidLocation(x, y):
                    gCost = aNode.g+1
                    if self.isObstacle(x,y):
                        gCost = 9999
                    neighbors.append(Node.makeNode(x,y, g = gCost))
        self.graph[aNode] = neighbors
        return self.graph[aNode]

    # return true if the given x and y values represent a valid location in the OccupancyGrid
    def isValidLocation(self, x, y):

        yes = self.isInGrid(x, y)# and not self.isObstacle(x, y) #comment out obstacle check to allow "obstacle nodes"
        # print "is valid location? ", (x,y), yes
        return yes

    # return true if the x and y values are within the bounds of the OccupancyGrid
    def isInGrid(self,x, y):
        return x in range(0,self.width) and y in range(0,self.height)

    # returns true if the x and y coordiantes represent a point with a high
    # probaility of being an obstacle
    def isObstacle(self, x, y):
        # builds borders for the "grid"
        listIndex = y * self.width + x
        yes = self.cellStates[listIndex] > 50
        return yes

    # returns the probability of the cell containing an obstacle
    def cellValue(self, node):
        listIndex = node.y * self.width + node.x
        return self.cellStates[listIndex]

    # converts the given node to its corrsponding point in the map frame
    def convertNodeToPoint(self, node):
        resolution = self.map.info.resolution
        offsetX = self.map.info.origin.position.x
        offsetY = self.map.info.origin.position.y
        xAdj = yAdj = 0.5
        point = Point()
        point.x = (node.x * resolution) + (xAdj * resolution) + offsetX
        point.y = (node.y * resolution) + (yAdj * resolution) + offsetY
        return point
 
