#include "ros_subscriber.h"

#include "exec_env/global_mutex.h"
#include "core/execution/graph.h"
#include "core/core-dev.h"

using std::placeholders::_1;

using namespace djnn;

RosSubscriber::RosSubscriber (CoreProcess* parent, const string& n, const string& topic_name) :
  FatProcess (n),
  ExternalSource (n),
  _topic_name (topic_name),
  _msg (this, "message", "")
{
  _node = std::make_shared<rclcpp::Node>(n);

  finalize_construction (parent, n);
}

void
RosSubscriber::impl_activate ()
{
  subscription_ =_node->create_subscription<std_msgs::msg::String>(
      _topic_name, 10, std::bind(&RosSubscriber::receive_msg, this, _1));
  ExternalSource::start ();  
}

void
RosSubscriber::impl_deactivate ()
{
  // Here we should disable the subcription but it 
  // seems there is no way to do it properly
  // some insights here:
  // https://answers.ros.org/question/354792/rclcpp-how-to-unsubscribe-from-a-topic/
  subscription_.reset ();
  ExternalSource::please_stop ();
}

void 
RosSubscriber::receive_msg (const std_msgs::msg::String::SharedPtr msg) {
  RCLCPP_INFO(_node->get_logger(), "I heard: '%s'", msg->data.c_str());
  get_exclusive_access(DBG_GET);
  _msg.set_value (msg->data.c_str(), true);
  GRAPH_EXEC;
  release_exclusive_access(DBG_REL);
}

void
RosSubscriber::run () {
  rclcpp::spin(_node);
  rclcpp::shutdown();
}
