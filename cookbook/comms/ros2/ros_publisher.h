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
    MyRosPublisher (ParentProcess* parent, const string& n, const string& topic_name);
    ~MyRosPublisher() {}

    void impl_activate () override;
    void impl_deactivate () override;

    void run () override;
  
    void send_msg ();

  private:
    TextProperty _msg;
    SendMsgAction _action;
    Coupling _c_msg;
    rclcpp::Node::SharedPtr _node;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  };
