from rosbag import Bag
with Bag('pcl_bag0002.bag', 'w') as Y:
    for topic, msg, t in Bag('pcl_bag0001.bag'):
        Y.write('/kinect/depth_registered/points' if topic == '/camera/depth_registered/points' else topic, msg, t)
