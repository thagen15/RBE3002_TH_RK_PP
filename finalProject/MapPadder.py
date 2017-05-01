#!/usr/bin/python
import rospy, math
import rospy, tf, numpy, math, sys
from std_msgs.msg import String
from nav_msgs.msg import GridCells
from math import sqrt
from nav_msgs.msg import OccupancyGrid


def mapCallback(data):
	global pub_expMap

	expandFactor = int(math.ceil(rospy.get_param('expend', 0.2)/data.info.resolution))

	height = data.info.height
	width = data.info.width

	expMap = OccupancyGrid()
	expMap.info = data.info
	expMap.data = [0 for x in range(width*height)]
	for x in range(width*height):
		if data.data[x] < 0:
			expMap.data[x] = -1
		else:
			for n in getAdjacent(data, x, expandFactor):
				if n > 70:
					expMap.data[x] = 100
					break
		

	pub_expMap.publish(expMap)

def getAdjacent(data, node, expandFactor):
	width = data.info.width
	height = data.info.height
	neighbors = []

	for x in range(-expandFactor, expandFactor+1):
		for y in range(-expandFactor, expandFactor+1):
			a = node + x + width*y
			if a > 0 and a < width*height:
				neighbors.append(data.data[a])

	return neighbors

if __name__ == '__main__':

	global pub_expMap
	global map_sub

	rospy.init_node('Map Padder')
	
	pub_expMap = rospy.Publisher(rospy.get_param('output_map_topic', '/exp_map'), OccupancyGrid, latch=True)
	map_sub = rospy.Subscriber(rospy.get_param('input_map_topic', '/map'), OccupancyGrid, mapCallback, queue_size=10)
	rospy.spin()