/*
 *	djnn Smala compiler
 *
 *	The copyright holders for the contents of this file are:
 *		Ecole Nationale de l'Aviation Civile, France (2022)
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

import gui.widgets.StandAlonePushButton

_main_
Component root {
  Frame f ("my frame", 500, 500, 500, 500)
  Exit ex (0, 1)
  f.close -> ex
  mouseTracking = 1
  
  FillColor _(White)
  MultilineEditor ste (5, 0, $f.width - 100, $f.height, "", 1)
  2 =: ste.spaces_for_tab
  f.width - 20 =:> ste.width
  ste.line_height * ste.lines.size =:> ste.height
  "Console\n" =: ste.string_input

  StandAlonePushButton button ("Click to add line", 450, 100)
  f.width - button.width - 10 =:> button.x
  button.click->{"new line\n" =: ste.string_input}
  StandAlonePushButton button2 ("Click to clear", 450, 150)
  f.width - button2.width - 10 =:> button2.x
  button2.click->ste.clear
}

