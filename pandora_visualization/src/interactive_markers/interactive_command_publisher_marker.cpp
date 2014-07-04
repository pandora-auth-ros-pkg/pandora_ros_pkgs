#include <ros/ros.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Float64.h>

#include <interactive_markers/interactive_marker_server.h>

namespace pandora_visualization
{
  class InteractiveCommandPublisherMarker
  {
    private:
      ros::NodeHandle node_handle_;
      std::string name_;
      
      ros::Publisher x_publisher_;
      ros::Publisher y_publisher_;
      ros::Publisher z_publisher_;
      ros::Publisher roll_publisher_;
      ros::Publisher pitch_publisher_;
      ros::Publisher yaw_publisher_;
      
      interactive_markers::InteractiveMarkerServer server_;
      
      std::string parent_frame_;
      std::string x_topic_;
      std::string y_topic_;
      std::string z_topic_;
      std::string roll_topic_;
      std::string pitch_topic_;
      std::string yaw_topic_;

      std_msgs::Float64 x_command_;
      std_msgs::Float64 y_command_;
      std_msgs::Float64 z_command_;
      std_msgs::Float64 roll_command_;
      std_msgs::Float64 pitch_command_;
      std_msgs::Float64 yaw_command_;

      ros::Timer publish_timer_;
      double publish_period_;

      bool getParams();
      void createMarker();
      void processFeedback(
        const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
      void timerCallback(const ros::TimerEvent);
    public:
      InteractiveCommandPublisherMarker(std::string name);
      ~InteractiveCommandPublisherMarker();
  };

  InteractiveCommandPublisherMarker::InteractiveCommandPublisherMarker(
    std::string name) :
      name_(name),
      server_(name)
    // create an interactive marker server on the topic namespace
    // interactive_tf_publisher_marker
  {
    // get params from param server
    if(!getParams())
    {
      return;
    }

    createMarker();
    
    roll_command_.data = 0.0;
    pitch_command_.data = 0.0;
    yaw_command_.data = 0.0;
    
    publish_timer_ = node_handle_.createTimer(ros::Duration(publish_period_),
      boost::bind(
        &InteractiveCommandPublisherMarker::timerCallback, this, _1));
  }

  InteractiveCommandPublisherMarker::~InteractiveCommandPublisherMarker()
  {
  }

  bool InteractiveCommandPublisherMarker::getParams()
  {
    if (node_handle_.getParam(name_ + "/x_topic", x_topic_))
    {
      ROS_INFO_STREAM("Got param x_topic: " << x_topic_);
    }
    else
    {
      ROS_INFO_STREAM(
        "Failed to get param x_topic. No x command will be published");
    }

    if (node_handle_.getParam(name_ + "/y_topic", y_topic_))
    {
      ROS_INFO_STREAM("Got param y_topic: " << y_topic_);
    }
    else
    {
      ROS_INFO_STREAM(
        "Failed to get param y_topic. No y command will be published");
    }

    if (node_handle_.getParam(name_ + "/z_topic", z_topic_))
    {
      ROS_INFO_STREAM("Got param z_topic: " << z_topic_);
    }
    else
    {
      ROS_INFO_STREAM(
        "Failed to get param z_topic. No z command will be published");
    }

    if (node_handle_.getParam(name_ + "/roll_topic", roll_topic_))
    {
      ROS_INFO_STREAM("Got param roll_topic: " << roll_topic_);
    }
    else
    {
      ROS_INFO_STREAM(
        "Failed to get param roll_topic. No roll command will be published");
    }

    if (node_handle_.getParam(name_ + "/pitch_topic", pitch_topic_))
    {
      ROS_INFO_STREAM("Got param pitch_topic: " << pitch_topic_);
    }
    else
    {
      ROS_INFO_STREAM(
        "Failed to get param pitch_topic. No pitch command will be published");
    }

    if (node_handle_.getParam(name_ + "/yaw_topic", yaw_topic_))
    {
      ROS_INFO_STREAM("Got param yaw_topic: " << yaw_topic_);
    }
    else
    {
      ROS_INFO_STREAM(
        "Failed to get param yaw_topic. No yaw command will be published");
    }

    if (node_handle_.getParam(name_ + "/parent_frame", parent_frame_))
    {
      ROS_INFO_STREAM("Got param parent_frame: " << parent_frame_);
    }
    else
    {
      ROS_ERROR_STREAM("Failed to get param parent_frame using default: base_link");
      return false;
    }

    node_handle_.param(name_ + "/publish_period", publish_period_, 0.5);
    if (publish_period_ <= 0)
    {
      ROS_WARN_STREAM("publish_period must be a positive number. " <<
        publish_period_ << " was given. Using default: 0.5");
      publish_period_ = 0.5;
    }
    return true;
  }

  void InteractiveCommandPublisherMarker::createMarker()
  {
    // create an interactive marker for our server
    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header.frame_id = parent_frame_;
    int_marker.name = name_;
    int_marker.description = name_;

    // create a red sphere marker
    visualization_msgs::Marker sphere_marker;
    sphere_marker.type = visualization_msgs::Marker::SPHERE;
    sphere_marker.scale.x = 0.05;
    sphere_marker.scale.y = 0.05;
    sphere_marker.scale.z = 0.05;
    sphere_marker.color.r = 1.0;
    sphere_marker.color.g = 0.0;
    sphere_marker.color.b = 0.0;
    sphere_marker.color.a = 1.0;

    // create a non-interactive control which contains the box
    visualization_msgs::InteractiveMarkerControl sphere_control;
    sphere_control.always_visible = true;
    sphere_control.markers.push_back(sphere_marker);

    // add the control to the interactive marker
    int_marker.controls.push_back(sphere_control);

    // create publishers
    if (x_topic_ != "")
    {
      x_publisher_ = node_handle_.advertise<std_msgs::Float64>(x_topic_, 1);

      // create a control which will move the marker around x
      visualization_msgs::InteractiveMarkerControl move_x_control;
      move_x_control.name = "move_x";
      move_x_control.always_visible = true;
      move_x_control.interaction_mode =
        visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
      // add the control to the interactive marker
      int_marker.controls.push_back(move_x_control);
    }
    if (y_topic_ != "")
    {
      y_publisher_ = node_handle_.advertise<std_msgs::Float64>(y_topic_, 1);

      // create a control which will move the marker around y
      visualization_msgs::InteractiveMarkerControl move_y_control;
      move_y_control.name = "move_y";
      move_y_control.always_visible = true;
      move_y_control.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 1.57);
      move_y_control.interaction_mode =
        visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
      // add the control to the interactive marker
      int_marker.controls.push_back(move_y_control);
    }
    if (z_topic_ != "")
    {
      z_publisher_ = node_handle_.advertise<std_msgs::Float64>(z_topic_, 1);

      // create a control which will move the marker around z
      visualization_msgs::InteractiveMarkerControl move_z_control;
      move_z_control.name = "move_z";
      move_z_control.always_visible = true;
      move_z_control.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, -1.57, 0);
      move_z_control.interaction_mode =
        visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
      // add the control to the interactive marker
      int_marker.controls.push_back(move_z_control);
    }
    if (roll_topic_ != "")
    {
      roll_publisher_ = node_handle_.advertise<std_msgs::Float64>(roll_topic_, 1);

      // create a control which will rotate the marker around x
      visualization_msgs::InteractiveMarkerControl rotate_x_control;
      rotate_x_control.name = "rotate_x";
      rotate_x_control.always_visible = true;
      rotate_x_control.interaction_mode =
        visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
      // add the control to the interactive marker
      int_marker.controls.push_back(rotate_x_control);
    }
    if (pitch_topic_ != "")
    {
      pitch_publisher_ = node_handle_.advertise<std_msgs::Float64>(pitch_topic_, 1);

      // create a control which will rotate the marker around y
      visualization_msgs::InteractiveMarkerControl rotate_y_control;
      rotate_y_control.name = "rotate_y";
      rotate_y_control.always_visible = true;
      rotate_y_control.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 1.57);
      rotate_y_control.interaction_mode =
        visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
      // add the control to the interactive marker
      int_marker.controls.push_back(rotate_y_control);
    }
    if (yaw_topic_ != "")
    {
      yaw_publisher_ = node_handle_.advertise<std_msgs::Float64>(yaw_topic_, 1);

      // create a control which will rotate the marker around z
      visualization_msgs::InteractiveMarkerControl rotate_z_control;
      rotate_z_control.name = "rotate_z";
      rotate_z_control.always_visible = true;
      rotate_z_control.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, -1.57, 0);
      rotate_z_control.interaction_mode =
          visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;

      // add the control to the interactive marker
      int_marker.controls.push_back(rotate_z_control);
    }

    // add the interactive marker to our collection &
    // tell the server to call processFeedback() when feedback arrives for it
    server_.insert(int_marker,
      boost::bind(&InteractiveCommandPublisherMarker::processFeedback, this, _1));

    // 'commit' changes and send to all clients
    server_.applyChanges();
  }

  void InteractiveCommandPublisherMarker::processFeedback(
      const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
  {
    x_command_.data = feedback->pose.position.x;
    y_command_.data = feedback->pose.position.y;
    z_command_.data = feedback->pose.position.z;

    tf::Quaternion orientation;
    tf::quaternionMsgToTF(feedback->pose.orientation, orientation);
    tf::Matrix3x3 rotation(orientation);
    rotation.getRPY(roll_command_.data, pitch_command_.data, yaw_command_.data);
  }

  void InteractiveCommandPublisherMarker::timerCallback(
    const ros::TimerEvent)
  {
    if (x_topic_ != "")
    {
      x_publisher_.publish(x_command_);
    }
    if (y_topic_ != "")
    {
      y_publisher_.publish(y_command_);
    }
    if (z_topic_ != "")
    {
      z_publisher_.publish(z_command_);
    }
    if (roll_topic_ != "")
    {
      roll_publisher_.publish(roll_command_);
    }
    if (pitch_topic_ != "")
    {
      pitch_publisher_.publish(pitch_command_);
    }
    if (yaw_topic_ != "")
    {
      yaw_publisher_.publish(yaw_command_);
    }
  }
}  // namespace pandora_visualization

int main(int argc, char** argv)
{
  ros::init(argc, argv, argv[1]);
  std::string name = argv[1];
  pandora_visualization::InteractiveCommandPublisherMarker
    interactiveCommandPublisherMarker(name);

  ros::spin();
}
