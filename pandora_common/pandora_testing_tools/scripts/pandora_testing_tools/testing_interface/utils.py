import math
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Point


def distance(a, b):

    return math.sqrt((a.x - b.x)**2 + (a.y - b.y)**2 + (a.z - b.z)**2)

def distance_keypoints(a, b):

    return math.sqrt((a.x - b.x)**2 + (a.y - b.y)**2)

def isOrientationReversed(a, b):

    roll, pitch, yaw = euler_from_quaternion([a.x, a.y, a.z, a.w])
    reversed_a = quaternion_from_euler(roll, pitch, yaw + math.pi)
    distance = math.sqrt((reversed_a[0] - b.x)**2 + (reversed_a[1] - b.y)**2 + (reversed_a[2] - b.z)**2 + (reversed_a[3] - b.w)**2)
    return distance == 0

def isPositionGrounded(a, b):

    a.z = 0
    return distance(a, b) == 0

def direction(a, b):

    dire = Point()
    norm = distance(a, b)
    dire.x = (b.x - a.x)/norm
    dire.y = (b.y - a.y)/norm
    dire.z = (b.z - a.z)/norm
    return dire
