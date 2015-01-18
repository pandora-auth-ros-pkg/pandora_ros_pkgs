#include <ros/ros.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include <interactive_markers/interactive_marker_server.h>

namespace pandora_visualization
{
  class InteractiveTfPublisherMarker
  {
    private:
      ros::NodeHandle node_handle_;
      std::string name_;
      tf::TransformBroadcaster tf_broadcaster_;
      tf::Transform transform_;

      interactive_markers::InteractiveMarkerServer server_;

      std::string parent_frame_;
      std::string marker_frame_;

      ros::Timer publish_timer_;
      double publish_period_;

      bool getParams();
      void createMarker();
      void processFeedback(
        const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
      void timerCallback(const ros::TimerEvent);
    public:
      InteractiveTfPublisherMarker(std::string name);
      ~InteractiveTfPublisherMarker();
  };

  InteractiveTfPublisherMarker::InteractiveTfPublisherMarker(
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

    // initialize transform_
    transform_.setOrigin(tf::Vector3(0, 0, 0));
    transform_.setRotation(tf::createIdentityQuaternion());

    publish_timer_ = node_handle_.createTimer(ros::Duration(publish_period_),
      boost::bind(
        &InteractiveTfPublisherMarker::timerCallback, this, _1));
  }

  InteractiveTfPublisherMarker::~InteractiveTfPublisherMarker()
  {
  }

  bool InteractiveTfPublisherMarker::getParams()
  {
    if (node_handle_.getParam(name_ + "/parent_frame", parent_frame_))
    {
      ROS_INFO_STREAM("Got param parent_frame: " << parent_frame_);
    }
    else
    {
      ROS_INFO_STREAM("Failed to get param parent_frame using default: base_link");
      parent_frame_ = "base_link";
    }

    if (node_handle_.getParam(name_ + "/marker_frame", marker_frame_))
    {
      ROS_INFO_STREAM("Got param marker_frame: " << marker_frame_);
    }
    else
    {
      ROS_INFO_STREAM("Failed to get param marker_frame using default: marker");
      marker_frame_ = "marker_frame";
    }
    node_handle_.param("publish_period", publish_period_, 0.5);
    if (publish_period_ <= 0)
    {
      ROS_WARN_STREAM("publish_period must be a positive number. " <<
        publish_period_ << " was given. Using default: 0.5");
      publish_period_ = 0.5;
    }
    return true;
  }


  void InteractiveTfPublisherMarker::createMarker()
  {
    // create an interactive marker for our server
    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header.frame_id = parent_frame_;
    int_marker.name = name_;
    int_marker.description = name_;

    // create a red sphere marker
    visualization_msgs::Marker sphere_marker;
    sphere_marker.type = visualization_msgs::Marker::SPHERE;
    sphere_marker.scale.x = 0.1;
    sphere_marker.scale.y = 0.1;
    sphere_marker.scale.z = 0.1;
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

    // create a control which will move the marker around x
    visualization_msgs::InteractiveMarkerControl move_x_control;
    move_x_control.name = "move_x";
    move_x_control.always_visible = true;
    move_x_control.interaction_mode =
        visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
        
    // create a control which will move the marker around y
    visualization_msgs::InteractiveMarkerControl move_y_control;
    move_y_control.name = "move_y";
    move_y_control.always_visible = true;
    move_y_control.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 1.57);
    move_y_control.interaction_mode =
        visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;

    // create a control which will move the marker around z
    visualization_msgs::InteractiveMarkerControl move_z_control;
    move_z_control.name = "move_z";
    move_z_control.always_visible = true;
    move_z_control.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, -1.57, 0);
    move_z_control.interaction_mode =
        visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;

    // create a control which will rotate the marker around x
    visualization_msgs::InteractiveMarkerControl rotate_x_control;
    rotate_x_control.name = "rotate_x";
    rotate_x_control.always_visible = true;
    rotate_x_control.interaction_mode =
      visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;

    // create a control which will rotate the marker around y
    visualization_msgs::InteractiveMarkerControl rotate_y_control;
    rotate_y_control.name = "rotate_y";
    rotate_y_control.always_visible = true;
    rotate_y_control.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 1.57);
    rotate_y_control.interaction_mode =
      visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;

    // create a control which will rotate the marker around z
    visualization_msgs::InteractiveMarkerControl rotate_z_control;
    rotate_z_control.name = "rotate_z";
    rotate_z_control.always_visible = true;
    rotate_z_control.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, -1.57, 0);
    rotate_z_control.interaction_mode =
        visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;

    // add the control to the interactive marker
    int_marker.controls.push_back(move_x_control);
    int_marker.controls.push_back(move_y_control);
    int_marker.controls.push_back(move_z_control);
    int_marker.controls.push_back(rotate_x_control);
    int_marker.controls.push_back(rotate_y_control);
    int_marker.controls.push_back(rotate_z_control);

    // add the interactive marker to our collection &
    // tell the server to call processFeedback() when feedback arrives for it
    server_.insert(int_marker,
      boost::bind(&InteractiveTfPublisherMarker::processFeedback, this, _1));

    // 'commit' changes and send to all clients
    server_.applyChanges();
  }

  void InteractiveTfPublisherMarker::processFeedback(
      const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
  {
    transform_.setOrigin(
      tf::Vector3(
        feedback->pose.position.x,
        feedback->pose.position.y,
        feedback->pose.position.z));
    transform_.setRotation(
      tf::Quaternion(
        feedback->pose.orientation.x,
        feedback->pose.orientation.y,
        feedback->pose.orientation.z,
        feedback->pose.orientation.w));
  }

  void InteractiveTfPublisherMarker::timerCallback(
    const ros::TimerEvent)
  {
    tf_broadcaster_.sendTransform(
      tf::StampedTransform(
        transform_, ros::Time::now(), parent_frame_ , marker_frame_));
  }
}  // namespace pandora_visualization

int main(int argc, char** argv)
{
  ros::init(argc, argv, argv[1]);
  std::string name = argv[1];
  pandora_visualization::InteractiveTfPublisherMarker
    interactiveTfPublisherMarker(name);

  ros::spin();
}
