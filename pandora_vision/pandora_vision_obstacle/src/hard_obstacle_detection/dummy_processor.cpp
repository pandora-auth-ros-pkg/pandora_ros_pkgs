#include <string>
#include "sensor_msgs/Image.h"
#include <limits>
#include <sensor_msgs/image_encodings.h>

#include "pandora_vision_obstacle/hard_obstacle_detection/dummy_processor.h"

namespace pandora_vision
{
namespace pandora_vision_obstacle
{
  DummyProcessor::DummyProcessor() : it(nh)
  {
    detector_.reset(new HardObstacleDetector("detector", nh));
    if (detector_ == NULL)
    {
      ROS_FATAL("Could not initialize Hard Obstacle Detector!");
      ROS_BREAK();
    }
    sub = it.subscribe("/elevation_map", 1,
      &DummyProcessor::imageCallback, this);
    pub = it.advertise("/traversability_map", 1);
  }

  void DummyProcessor::displayElevationMap(cv::Mat inputImage)
  {
    cv::namedWindow("Elevation Map Image", cv::WINDOW_NORMAL);
    cv::Mat elevationMapImg(inputImage.size(), CV_8UC1);
    cv::Mat colorMapImg;

    // Find the known areas in the map.
    cv::Mat mask = inputImage != -std::numeric_limits<double>::max();

    // Normalize only the map elements that correspond to known cells.
    cv::normalize(inputImage, elevationMapImg, 0, 1, cv::NORM_MINMAX, -1, mask);

    elevationMapImg.convertTo(elevationMapImg, CV_8UC1, 255);
    cv::applyColorMap(elevationMapImg, colorMapImg, cv::COLORMAP_JET);

    // Set all unknown areas to 0 so that they appear as black.
    cv::bitwise_not(mask, mask);
    colorMapImg.setTo(0.0, mask);

    cv::imshow("Elevation Map Image", colorMapImg);
    cv::waitKey(5);
    return;
  }

  void DummyProcessor::imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_64FC1);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }

    CVMatStampedPtr inputImagePtr( new CVMatStamped );
    inputImagePtr->image = cv_ptr->image;
    inputImagePtr->header = cv_ptr->header;

    CVMatStampedPtr outputImagePtr( new CVMatStamped );

    displayElevationMap(inputImagePtr->image);
    bool flag = process(inputImagePtr, outputImagePtr);

    cv_bridge::CvImage out_msg;
    out_msg.header   = outputImagePtr->header; // Same timestamp and tf frame as input image
    out_msg.encoding = sensor_msgs::image_encodings::TYPE_8UC1; // Or whatever
    out_msg.image    = outputImagePtr->image; // Your cv::Mat
    if (flag)
      pub.publish(out_msg.toImageMsg());
  }

  bool DummyProcessor::process(const CVMatStampedConstPtr& input,
      const CVMatStampedPtr& output)
  {
    output->header = input->getHeader();
    output->image = detector_->startDetection(input->getImage());

    return true;
  }

}
}
