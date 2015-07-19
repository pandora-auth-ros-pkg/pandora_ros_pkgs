#include "pandora_vision_obstacle/hard_obstacle_detection/dummy_processor.h"

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "dummy_hard_obstacle_node");
  pandora_vision::pandora_vision_obstacle::DummyProcessor dummyProcessor;
  ros::spin();
  return 0;
}
