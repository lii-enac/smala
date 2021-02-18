/*
*  djnn Smala compiler
*
*  The copyright holders for the contents of this file are:
*    Ecole Nationale de l'Aviation Civile, France (2020-2021)
*  See file "license.terms" for the rights and conditions
*  defined by copyright holders.
*
*
*  Contributors:
*    
*     Stephane Conversy <stephane.conversy@enac.fr>
*     Mathieu Poirier <mathieu.poirier@enac.fr>
*/

use core
use base
use display
use gui

_main_
Component root {
  Frame f ("transparent windows", 0, 0, 600, 600)
  Exit ex (0, 1)
  f.close -> ex

  FillColor w (White)
  Text txt (100, 300, "Click anywhere on the Window to modify background opacity and  color")
  FillColor g (0, 255, 0)
  Rectangle r (10, 10, 50, 50)
  FillColor g (255, 0, 0)
  Circle  c (60, 60, 30)

  // init the background_opacity and color before the frame activation
  f.background_opacity = 0.5
  // f.background_color.r = 0
  // f.background_color.g = 0
  // f.background_color.b = 0
 
  f.press.x / 900 => f.background_opacity
  (f.press.x / 900) * 255 =>  f.background_color.b

  Timer t(2000)
  t.end -> { 900 =: f.width }
}