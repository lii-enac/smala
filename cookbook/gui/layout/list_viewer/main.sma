/*
 *	djnn Smala compiler
 *
 *	The copyright holders for the contents of this file are:
 *		Ecole Nationale de l'Aviation Civile, France (2021)
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
use gui


import gui.widgets.ListViewer

import LeftPanel
import SimpleFlightView
import FullFlightView

_native_code_
%{
using namespace std;

%}
_main_
Component root {
  Frame f ("frame", 0, 0, 600, 500)
  Exit ex (0, 1)
  f.close->ex
  LinearGradient lg (0, 0, 600, 500, 0, 0) {
    GradientStop _ (255, 167, 36, 1, 0)
    GradientStop _ (255, 197, 102, 1, 1)
  }

  NoOutline _
  Rectangle bg (0, 0, 0, 0, 0, 0)
  f.height =:> lg.y2, bg.height
  f.width =:> lg.x2, bg.width

  mouseTracking = 1 

  List flight_list
  LeftPanel left_panel (f, flight_list)

  Component prototype (1) {
    SimpleFlightView vview
    FullFlightView fullview
  }
  ListViewer full_list_viewer (flight_list, prototype.fullview)
  left_panel.width + 10 =:> full_list_viewer.x
  full_list_viewer.y = 1

  Frame small_view ("small view", 600, 0, 600, 200)
  ListViewer small_list_viewer (flight_list, prototype.vview)
  small_list_viewer.y = 2
  small_list_viewer.x = 2
  small_list_viewer.orientation = 1
 }
