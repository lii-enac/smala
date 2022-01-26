use core
use base

_native_code_
%{

#if 1

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

#include "exec_env/global_mutex.h"
#include "core/execution/graph.h"

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber(djnn::Process* c)
    : Node("minimal_subscriber"), c_(c)
    {
      subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    }

  private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
      get_exclusive_access(DBG_GET);
      auto * t = dynamic_cast<djnn::TextProperty*>(c_->get_parent()->find_child("msg"));
      t->set_value (msg->data.c_str(), true);
      release_exclusive_access(DBG_REL);
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    djnn::Process *c_;
};

#endif


static void
ros_async (Process* c)
{
  rclcpp::spin(std::make_shared<MinimalSubscriber>(c));
  rclcpp::shutdown();
}

%}


_define_
RosSubscriber ()
{
    //init_subscriber ()
    NativeAsyncAction na(ros_async, this, 0)
    String msg ("")
}