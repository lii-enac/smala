/*
*  djnn Smala compiler
*
*  The copyright holders for the contents of this file are:
*    Ecole Nationale de l'Aviation Civile, France (2022-2023)
*  See file "license.terms" for the rights and conditions
*  defined by copyright holders.
*
*
*  Contributors:
*     Mathieu Magnaudet <mathieu.magnaudet@enac.fr>
*
*/
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

class RosSubscriber : public FatProcess, public ExternalSource
  {
  public:
    RosSubscriber (CoreProcess* parent, const string& n, const string& topic_name);
    ~RosSubscriber() {}

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
