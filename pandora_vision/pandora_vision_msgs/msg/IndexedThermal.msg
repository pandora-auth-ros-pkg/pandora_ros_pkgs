Header header

# The image acquired from thermal sensor
sensor_msgs/Image thermalImage

# The calibated temperature values in each pixel with 20 - 50 range
std_msgs/Float32MultiArray temperatures

# An index that informs thermal node where it must publish its result
string thermalIndex
