/*
*  djnn Smala compiler
*
*  The copyright holders for the contents of this file are:
*    Ecole Nationale de l'Aviation Civile, France (2020)
*  See file "license.terms" for the rights and conditions
*  defined by copyright holders.
*
*
*  Contributors:
*    
*     Mathieu Magnaudet <mathieu.magnaudet@enac.fr>
*/

use core
use base
use display
use gui

import gui.keyboard.ControlKey

_main_
Component root {
  Frame f ("simplest", 0, 0, 600, 600)
  Exit ex (0, 1)
  f.close -> ex

  Text explain (100, 50, "Press any regular key to display its text value.")
  Text explain2 (100, 64, "Then test the combination ctrl-p (or cmd-p on mac) to display a message.")
  Text explain3 (100, 78, "Finally use ctrl-q (or cmd-q) to quit.")

  ControlKey ck (f, DJN_Key_P)
  ControlKey ck2 (f, DJN_Key_Q)
  ck2.press->ex

  Text t (100, 150, "")
  f.key\-pressed_text =:> t.text

  Text t2 (100, 170, "")
  ck.press -> { "ctrl P - pressed" =: t2.text}
  ck.release -> { "ctrl P - released" =: t2.text}

}
