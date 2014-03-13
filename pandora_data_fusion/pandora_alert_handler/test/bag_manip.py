import roslib
roslib.load_manifest('data_fusion')
import rospy
import rosbag
import sys


def renameFilter(topic,filterDict):
	if topic in filterDict:
		return filterDict[topic]
	else:
		return topic

inputBag = sys.argv[1]

outputBag = inputBag + 'filtered'



filterDict={ 
			'/slam/slamMap/result' : '/slam/slamMapMsg'
			}

with rosbag.Bag(outputBag, 'w') as outbag:
	for topic, msg, t in rosbag.Bag(inputBag).read_messages():		
		outbag.write(renameFilter(topic,filterDict),msg,t)

