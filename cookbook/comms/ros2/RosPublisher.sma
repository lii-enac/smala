
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


#include "core/ontology/process.h"
#include "core/ontology/coupling.h"
#include "core/control/action.h"
#include "core/tree/spike.h"
#include "core/tree/double_property.h"
#include "core/tree/text_property.h"
#include "exec_env/external_source.h"

using namespace djnn;

class MyRosPublisher : public FatProcess, public ExternalSource
  {
  private:

    /* SEND ACTION */
    class SendMsgAction : public Action
    {
    public:
      SendMsgAction (ParentProcess* parent, const string& name) :
        Action (parent, name) {};
    
      virtual ~SendMsgAction () {}
      void impl_activate () override { ((MyRosPublisher*)get_parent())->send_msg (); }
    };
  public:
    MyRosPublisher (ParentProcess* parent, const string& n) :
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
    ~MyRosPublisher() {}

    void impl_activate () override {
      _c_msg.enable ();
      ExternalSource::start ();  
    }
    void impl_deactivate () override {
      _c_msg.disable ();
      ExternalSource::please_stop ();
    }

    void run () override {
      rclcpp::spin(_node);
      rclcpp::shutdown();
    }
  
    void send_msg () {
      auto message = std_msgs::msg::String();
      message.data = "Sending msg: " + _msg.get_value ();
      RCLCPP_INFO(_node->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_->publish(message);
    }

    void serialize (const string& format) override{}

  private:
    TextProperty _msg;
    SendMsgAction _action;
    Coupling _c_msg;
    rclcpp::Node::SharedPtr _node;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  };


#endif

%}

_define_
RosPublisher ()
{
 MyRosPublisher ros_pub
 msg aka ros_pub.message
}
