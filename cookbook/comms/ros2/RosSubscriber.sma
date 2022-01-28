use core
use base

import ros_subscriber

_define_
RosSubscriber ()
{
  MyRosSubscriber sub ("topic")
  msg aka sub.message
}