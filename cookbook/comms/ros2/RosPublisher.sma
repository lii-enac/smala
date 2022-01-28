
use core
use base

import ros_publisher

_define_
RosPublisher ()
{
 MyRosPublisher ros_pub ("topic")
 msg aka ros_pub.message
}
