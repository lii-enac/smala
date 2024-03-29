/*
 *	djnn Smala compiler
 *
 *	The copyright holders for the contents of this file are:
 *		Ecole Nationale de l'Aviation Civile, France (2017)
 *	See file "license.terms" for the rights and conditions
 *	defined by copyright holders.
 *
 *
 *	Contributors:
 *		Mathieu Magnaudet <mathieu.magnaudet@enac.fr>
 *
 */

use core
use base
use display
use gui

import TextLineEdit
import ros_subscriber
import ros_publisher

_native_code_
%{
#include "rclcpp/rclcpp.hpp"

static void
init_ros ()
{
  rclcpp::init(0,0); //argc, argv);
}
%}

_main_
Component root {
  init_ros ()

  Frame f ("my frame", 500, 500, 500, 500)
  Exit ex (0, 1)
  f.close -> ex
  mouseTracking = 1
  NoOutline _
  FillColor _ (50, 50, 50)
  Rectangle bkg (0, 0, 0, 0, 0, 0)
  f.width  =:> bkg.width
  f.height =:> bkg.height
  TextLineEdit tle (f, "Enter your text here", 100, 100)

  FillColor _ (200, 200, 200)
  FontSize _ (4, 20)
  Text t (100, 200, "")
  
  RosSubscriber sub ("topic")
  RosPublisher  ros_pub ("topic")
  tle.validated_text =:> ros_pub.message
  sub.message =:> t.text
  }
