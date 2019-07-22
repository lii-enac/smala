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
  Frame f ("MDPC scrollbar", 0, 0, 400, 800)
  Exit ex (0, 1)
  f.close -> ex

  Scrollbar sb(f)
  0 =: sb.transform.tx
  100 =: sb.transform.ty
  300 =: sb.transform.s
}

run root
run syshook
