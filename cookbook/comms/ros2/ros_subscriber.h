#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"


#include "core/ontology/process.h"
#include "core/ontology/coupling.h"
#include "core/control/action.h"
#include "core/tree/text_property.h"
#include "exec_env/external_source.h"

using namespace djnn;

class MyRosSubscriber : public FatProcess, public ExternalSource
  {
  public:
    MyRosSubscriber (ParentProcess* parent, const string& n, const string& topic_name);
    ~MyRosSubscriber() {}

    void impl_activate () override;
    void impl_deactivate () override;

    void run () override;
  
    void receive_msg (const std_msgs::msg::String::SharedPtr msg);

  private:
    const std::string _topic_name;
    TextProperty _msg;
    rclcpp::Node::SharedPtr _node;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  };
