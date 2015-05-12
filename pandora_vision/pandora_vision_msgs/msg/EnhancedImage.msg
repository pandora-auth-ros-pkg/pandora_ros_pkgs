Header header
# This message contains images from RGB and D sensors.
# transformed (scaled, rotated and translated) so they match at what
# they depict.
# This message most likely has come from a syncronizer node.
# Each sensor message has the same width and height, fovx and fovy
# (which is probably in our implementations rgb camera's fovx and fovy).
# Each sensor_msgs/Image has different Header.frame_id
# but the same Header.stamp.

# current depth image transformed and synced
sensor_msgs/Image depthImage
# current rgb image transformed and synced
sensor_msgs/Image rgbImage

# flag that indicates if depth analysis is possible
bool isDepth

pandora_vision_msgs/AreaOfInterest[] areasOfInterest
