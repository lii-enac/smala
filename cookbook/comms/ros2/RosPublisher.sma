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

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher(djnn::Process *c)
    : Node("minimal_publisher"), count_(0), c_(c)
    {
      publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      auto message = std_msgs::msg::String();
      get_exclusive_access(DBG_GET);
      auto * t = dynamic_cast<djnn::TextProperty*>(c_->get_parent()->find_child("msg"));
      const string edit_box_string = t->get_value ();
      GRAPH_EXEC;
      release_exclusive_access(DBG_REL);
      message.data = "Hello, world! " + edit_box_string + " " + std::to_string(count_++);
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
    djnn::Process *c_;
};

#endif

static void
ros_async (Process* c)
{
  rclcpp::spin(std::make_shared<MinimalPublisher>(c));
  rclcpp::shutdown();
}


%}

_define_
RosPublisher ()
{
 //init_publisher ()
 NativeAsyncAction na(ros_async, this, 0)
 String msg ("")
}