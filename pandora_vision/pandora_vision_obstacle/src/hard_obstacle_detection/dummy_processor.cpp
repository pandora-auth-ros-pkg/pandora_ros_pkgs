#include <string>
#include "sensor_msgs/Image.h"
#include <limits>
#include <sensor_msgs/image_encodings.h>
#include "nav_msgs/OccupancyGrid.h"

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
    traversabilityMapPub_ = nh.advertise<nav_msgs::OccupancyGrid>("/vision/dummy_traversability_map", 1);
    mapSub_ = nh.subscribe("/slam/map", 1, &DummyProcessor::updateMap, this);

    map_const_ptr_.reset();

    nh.param<int>("unknown_value", UNKNOWN_VALUE, 51);
    nh.param<double>("mat_resolution", MAT_RESOLUTION, 0.02);
    nh.param<std::string>("local_frame", LOCAL_FRAME, "/base_footprint");
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
    struct timeval startwtime, endwtime;
    displayElevationMap(inputImagePtr->image);

    bool flag = process(inputImagePtr, outputImagePtr);

    nav_msgs::OccupancyGridPtr outputMapPtr;
    outputMapPtr.reset(new nav_msgs::OccupancyGrid);
    bool navFlag = postProcess(outputImagePtr, outputMapPtr);

    cv_bridge::CvImage out_msg;
    out_msg.header   = outputImagePtr->header;
    out_msg.encoding = sensor_msgs::image_encodings::TYPE_8UC1; // Or whatever
    out_msg.image    = outputImagePtr->image;
    if (flag)
      pub.publish(out_msg.toImageMsg());
    if (navFlag)
      traversabilityMapPub_.publish(*outputMapPtr);
  }

  bool DummyProcessor::process(const CVMatStampedConstPtr& input,
      const CVMatStampedPtr& output)
  {
    output->header = input->getHeader();
    output->image = detector_->startDetection(input->getImage());

    return true;
  }

  void DummyProcessor::updateMap(const nav_msgs::OccupancyGridConstPtr& mapConstPtr)
  {
    ROS_INFO("[%s]: Inside Map callback!", getName().c_str());
    map_const_ptr_ = mapConstPtr;
  }

  bool DummyProcessor::postProcess(const CVMatStampedConstPtr& input, const nav_msgs::OccupancyGridPtr& output)
  {
    if (map_const_ptr_ == NULL)
    {
      ROS_ERROR("[%s] Map pointer is still uninitialized!", this->getName().c_str());
      return false;
    }
    ROS_INFO("[%s] postprocessing to costmap", this->getName().c_str());

    output->header.frame_id = map_const_ptr_->header.frame_id;
    output->header.stamp = input->getHeader().stamp;
    output->info = map_const_ptr_->info;

    output->data.clear();
    output->data.resize(map_const_ptr_->data.size(), UNKNOWN_VALUE);

    // Get robot base footprint transform
    tf::StampedTransform baseTransform;
    try
    {
      tfListener_.waitForTransform(LOCAL_FRAME, output->header.frame_id,
          output->header.stamp, ros::Duration(0.2));
      tfListener_.lookupTransform(LOCAL_FRAME, output->header.frame_id,
          output->header.stamp, baseTransform);
    }
    catch (const tf::TransformException& ex)
    {
      ROS_ERROR("[%s]: Post Process transform Exception : %s", getName().c_str(), ex.what());
      return false;
    }

    tf::Quaternion baseOrientation;
    baseTransform.getBasis().getRotation(baseOrientation);
    tf::Vector3 origin = baseTransform.getOrigin();

    double yawDiff = tf::getYaw(map_const_ptr_->info.origin.orientation) -
      tf::getYaw(baseOrientation);
    double xDiff = map_const_ptr_->info.origin.position.x - origin[0];
    double yDiff = map_const_ptr_->info.origin.position.y - origin[1];

    int trans_ii, trans_jj;
    double x, y, xn, yn;
    cv::Mat flippedMap;
    cv::flip(input->image, flippedMap, 0);
    for (int ii = 0; ii < flippedMap.cols; ++ii) {
      for (int jj = 0; jj < flippedMap.rows; ++jj) {
        trans_ii = ii - flippedMap.cols / 2;
        trans_jj = jj - flippedMap.rows / 2;
        x = trans_ii * MAT_RESOLUTION;
        y = trans_jj * MAT_RESOLUTION;
        xn = cos(yawDiff) * x - sin(yawDiff) * y - xDiff;
        yn = sin(yawDiff) * x + cos(yawDiff) * y - yDiff;
        int coords = static_cast<int>(round(xn / output->info.resolution)) +
          static_cast<int>(round(yn / output->info.resolution)) * output->info.width;
        if (coords >= output->data.size() || coords < 0)
        {
          ROS_WARN("[%s] Error resizing to: %d\nCoords Xn: %f, Yn: %f\n",
              this->getName().c_str(), static_cast<int>(map_const_ptr_->data.size()), xn, yn);
        }
        else
        {
          output->data[coords] = static_cast<int8_t>(flippedMap.at<uchar>(jj, ii));
          obstacleDilation(output, 1, coords);
        }
      }
    }

    return true;
  }

  void
  DummyProcessor::
  obstacleDilation(const nav_msgs::OccupancyGridPtr& output, int steps, int coords)
  {
    if (steps == 0)
      return;

    int8_t cell = output->data[coords];

    if (cell != 0 && cell != UNKNOWN_VALUE)  // That's foreground
    {
      // Check for all adjacent
      if (output->data[coords + output->info.width + 1] == 0)
      {
        output->data[coords + output->info.width + 1] = cell;
        obstacleDilation(output, steps - 1, coords + output->info.width + 1);
      }
      if (output->data[coords + output->info.width] == 0)
      {
        output->data[coords + output->info.width] = cell;
      }
      if (output->data[coords + output->info.width - 1] == 0)
      {
        output->data[coords + output->info.width - 1] = cell;
        obstacleDilation(output, steps - 1, coords + output->info.width - 1);
      }
      if (output->data[coords + 1] == 0)
      {
        output->data[coords + 1] = cell;
      }
      if (output->data[coords - 1] == 0)
      {
        output->data[coords - 1] = cell;
      }
      if (output->data[coords - output->info.width + 1] == 0)
      {
        output->data[coords - output->info.width + 1] = cell;
        obstacleDilation(output, steps - 1, coords - output->info.width + 1);
      }
      if (output->data[coords - output->info.width] == 0)
      {
        output->data[coords - output->info.width] = cell;
      }
      if (output->data[coords - output->info.width - 1] == 0)
      {
        output->data[coords - output->info.width - 1] = cell;
        obstacleDilation(output, steps - 1, coords - output->info.width - 1);
      }
    }
  }

}
}
