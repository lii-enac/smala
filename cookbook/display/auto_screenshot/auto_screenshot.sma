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

import gui.interactors.SimpleDrag

_main_
Component root {
  Frame f ("screenshot", 0, 0, 600, 600)
  Exit ex (0, 1)
  f.close -> ex

  Rectangle r(10,10,50,50)
  Ref toDrag (null)
  r =: toDrag
  SimpleDrag _ (toDrag, f)

  WallClock wc
  f.refreshed -> {
    "screenshot-" + wc.state_text + ".png" =: f.screenshot_path
  }
  f.refreshed -> f.screenshot
  
}