#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

def talker():

    pub = rospy.Publisher('/slam/occupancyGridMap', OccupancyGrid , queue_size=10)
    
    pub2 = rospy.Publisher('robot_trajectory', Path , queue_size=10)
    pub3 = rospy.Publisher('/data_fusion/sensor_coverage/coverage_map'
       , OccupancyGrid,queue_size = 10)
    
    rospy.init_node('talker', anonymous = False)
    r = rospy.Rate(1) # 1 hz
    my_map = create_simple_map()
    my_path = create_simple_path()
    while not rospy.is_shutdown():

        rospy.loginfo("Publishing a Simple Map")

        pub.publish(my_map)
        pub2.publish(my_path)
        pub3.publish(my_map)
        r.sleep()

def create_simple_map():
  
  simple_map = OccupancyGrid()
  simple_map.info.width = 200
  simple_map.info.height = 200
  for i in range(200):
      for j in range(200):
          if (i == 0 or i ==50 or i ==100):
              simple_map.data.append(50)
          else :
              simple_map.data.append(127)
      
  return simple_map
def create_simple_path():
  
  simple_path = Path()
  for i in range(100):
    
    Point = PoseStamped()
    Point.pose.position.x=(i+50)
    Point.pose.position.y=(i+50)
    Point.pose.position.z=(50)
    
    simple_path.poses.append(Point)
    
  return simple_path;
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass
