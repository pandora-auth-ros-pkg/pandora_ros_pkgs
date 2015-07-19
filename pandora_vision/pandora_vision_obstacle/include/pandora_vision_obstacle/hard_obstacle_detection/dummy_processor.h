#include <cv_bridge/cv_bridge.h>
#include "pandora_vision_common/cv_mat_stamped.h"
#include <image_transport/image_transport.h>
#include "pandora_vision_obstacle/hard_obstacle_detection/hard_obstacle_detector.h"

namespace pandora_vision
{
namespace pandora_vision_obstacle
{
  class DummyProcessor
  {
    public:
      DummyProcessor();

      void displayElevationMap(cv::Mat elevationMapImg);
    public:
      virtual bool process(const CVMatStampedConstPtr& input,
          const CVMatStampedPtr& output);
      void imageCallback(const sensor_msgs::ImageConstPtr& msg);


    private:
      ros::NodeHandle nh;
      image_transport::ImageTransport it;
      image_transport::Subscriber sub;
      image_transport::Publisher pub;

      boost::shared_ptr<HardObstacleDetector> detector_;
  };
}
}
