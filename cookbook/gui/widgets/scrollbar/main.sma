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
*    Stephane Conversy <stephane.conversy@enac.fr>
*
*/

use core
use base
use gui

import Scrollbar

_main_
Component root {
  Frame f ("MDPC scrollbar", 0, 0, 800, 800)
  Exit ex (0, 1)
  f.close -> ex

  //Scaling _(1.1,1.1,0,0)

  Scrollbar sb(f)
  //50 =: sb.transform.tx
  //75 =: sb.transform.ty
  //300 =:> sb.transform.s
  //50 =: sb.width
  //50 =: sb.arrow_height

  TextPrinter tp
  //"low:   " + isString(sb.model.low) =:> tp.input
  //"high:  " + isString(sb.model.high) =:> tp.input
  //"delta: " + isString(sb.model.delta) =:> tp.input
  //"--" =:> tp.input
}

run root
run syshook
