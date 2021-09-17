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

//import gui.shape.text
import gui.widgets.IWidget

_define_
Label (Process container, string _label, double x_, double y_) inherits IWidget (container) {
  Translation t (x_, y_)

  /*----- interface -----*/
  x aka t.tx
  y aka t.ty
  /*----- interface -----*/

  FillColor fc (#323232)
  Translation offset (0, 0)
  Text ui (0, 0, _label)
  text aka ui.text
  this.height/2 + ui.ascent/2 - 1 =:> offset.ty

  ui.width =:> this.min_width
  ui.height =:> this.min_height
}
