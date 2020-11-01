/*
*  djnn Smala compiler
*
*  The copyright holders for the contents of this file are:
*    Ecole Nationale de l'Aviation Civile, France (2017)
*  See file "license.terms" for the rights and conditions
*  defined by copyright holders.
*
*
*  Contributors:
*    
*     Stephane Conversy <stephane.conversy@enac.fr>
*/

use core
use base
use display
use gui

_main_
Component root {
  Frame f ("screenshot", 0, 0, 600, 600)
  Exit ex (0, 1)
  f.close -> ex

  FillColor g(0,255,0)
  Rectangle r(10,10,50,50)
  "screenshot" =: f.screenshot_path // the implementation chooses the image file format
  r.press -> f.screenshot
}