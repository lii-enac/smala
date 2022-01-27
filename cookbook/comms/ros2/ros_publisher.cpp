#if 1

#include "ros_publisher.h"

using namespace djnn;

MyRosPublisher::MyRosPublisher (ParentProcess* parent, const string& n) :
  FatProcess (n),
  ExternalSource (n),
  _msg (this, "message", ""),
  _action (this, "send"),
  _c_msg (&_msg, ACTIVATION, &_action, ACTIVATION)
{
  _node = std::make_shared<rclcpp::Node>("MinimalPublisher");
  publisher_ =_node->create_publisher<std_msgs::msg::String>("topic", 10);
  finalize_construction (parent, n);
}

void
MyRosPublisher::impl_activate ()
{
  _c_msg.enable ();
  ExternalSource::start ();  
}

void
MyRosPublisher::impl_deactivate ()
{
  _c_msg.disable ();
  ExternalSource::please_stop ();
}

void 
MyRosPublisher::send_msg () {
  auto message = std_msgs::msg::String();
  message.data = "Sending msg: " + _msg.get_value ();
  RCLCPP_INFO(_node->get_logger(), "Publishing: '%s'", message.data.c_str());
  publisher_->publish(message);
}

void
MyRosPublisher::run () {
  rclcpp::spin(_node);
  rclcpp::shutdown();
}

#endif