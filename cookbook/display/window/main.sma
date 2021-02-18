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
  Frame f ("transparent windows", 0, 0, 600, 600, false)
  Exit ex (0, 1)
  f.close -> ex

  FillColor g (0, 255, 0)
  Rectangle r (10, 10, 50, 50)
  FillColor g (255, 0, 0)
  Circle  c (60, 60, 30)
  //0.6 =: f.opacity

  Timer t(2000)
  t.end -> {
    900 =: f.width
  }
}