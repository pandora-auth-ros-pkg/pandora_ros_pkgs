#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
#include "pandora_vision_common/cv_mat_stamped.h"
#include <nav_msgs/OccupancyGrid.h>
#include <image_transport/image_transport.h>
#include "pandora_vision_obstacle/hard_obstacle_detection/hard_obstacle_detector.h"
#include "tf/transform_listener.h"

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

      void updateMap(const nav_msgs::OccupancyGridConstPtr& mapConstPtr);

      bool postProcess(const CVMatStampedConstPtr& input, const nav_msgs::OccupancyGridPtr& output);

      std::string getName()
      {
        return std::string("Dummy Hard Obstacle Processor");
      }

      void obstacleDilation(const nav_msgs::OccupancyGridPtr& output, int steps, int coords);
    private:
      ros::NodeHandle nh;
      image_transport::ImageTransport it;
      image_transport::Subscriber sub;
      image_transport::Publisher pub;
      ros::Subscriber mapSub_;
      ros::Publisher traversabilityMapPub_;
      nav_msgs::OccupancyGridConstPtr map_const_ptr_;
      int UNKNOWN_VALUE;
      double MAT_RESOLUTION;
      std::string LOCAL_FRAME;

      tf::TransformListener tfListener_;
      boost::shared_ptr<HardObstacleDetector> detector_;
  };
}
}
