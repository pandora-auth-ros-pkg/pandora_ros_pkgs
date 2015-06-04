#!/usr/bin/python


import sys
import os
import rosbag

def split_bag(input_bag_absolute, size):
    input_bag = os.path.basename(os.path.abspath(input_bag_absolute))
    bag_path = os.path.dirname(os.path.abspath(input_bag_absolute))
    bag_name = os.path.splitext(input_bag)
    bag_part_counter = 0
    out_bag_path = bag_path+"/"+bag_name[0]+"_pt"+str(bag_part_counter)+".bag"
    out_bag = rosbag.Bag(out_bag_path, 'w')
    print(bag_name[0]+"_pt"+str(bag_part_counter)+".bag")
    for topic, msg, t in rosbag.Bag(input_bag_absolute).read_messages():
        if int((os.path.getsize(out_bag_path) >> 20)) < int(size):
            print (os.path.getsize(out_bag_path) >> 20)
            print (size)
            out_bag.write(topic, msg, t)
        else:
            bag_part_counter += 1
            out_bag_path = bag_path+"/"+bag_name[0]+"_pt"+str(bag_part_counter)+".bag"
            out_bag = rosbag.Bag(out_bag_path, 'w')
            print(bag_name[0]+"_pt"+str(bag_part_counter)+".bag")


def main(argv=sys.argv):
    split_bag(argv[1], argv[2])
    print( "Usage: ./split_bag.py  bag_file size_of_chunk_in_MB")


if __name__ == "__main__":
    sys.exit(main())
