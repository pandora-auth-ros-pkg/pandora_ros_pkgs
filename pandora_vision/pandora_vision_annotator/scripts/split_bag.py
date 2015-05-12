#!/usr/bin/python

from ros import rosbag
import rospy
import sys


def SplitBag(split_msg, num_msgs,bagname,output1,output2,top1):
    print(bagname);
    num_msgs=int(num_msgs);
    split_msg= int (split_msg)
    count_up = 0
    count_down = split_msg
    with rosbag.Bag(output1, 'w') as outbag:

        for topic, msg, t in rosbag.Bag(bagname).read_messages(topics=[top1]):
            if count_down < 1:
                break
            count_down = count_down - 1
            outbag.write(topic, msg, t)
    
    with rosbag.Bag(output2, 'w') as outbag:

        for topic, msg, t in rosbag.Bag(bagname).read_messages(topics=[top1]):
            
            if( count_up  > split_msg):
                outbag.write(topic, msg, t)
            count_up= count_up + 1

            if count_up > num_msgs:
                break


if __name__ == "__main__":
    if len( sys.argv ) == 7:
        SplitBag(sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4], sys.argv[5], sys.argv[6])
    else:
        print( "Usage: ./split_bag.py  split_msg  num_msgs inputbagfilename outputbagfilename1 outputbagfilename2 topicname1")
