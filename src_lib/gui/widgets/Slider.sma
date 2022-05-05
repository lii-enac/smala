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
Slider () inherits IWidget () {
  /*----- interface -----*/
  Int value (0)
  /*----- interface -----*/

  Translation pos (0, 0)
  NoOutline _
  FillColor bg (White)
  Rectangle slider (0, 7, 140, 6, 3, 3)
  FillColor _ (#535353)
  Rectangle fill (0, 7, 0, 6, 3, 3)
  FillColor fg (#323232)
  Circle handle (10, 10, 10)


  //logic and constraints
  handle.cx =:> fill.width
  
  this.width =:> slider.width
  this.min_width = 140
  this.min_height = 20
  this.height / 2 - 10 =:> pos.ty

  Double coeff (1)
  slider.width/ (slider.width-20) =:> coeff
  BoundedValue bv (10, 130, 10)
  slider.width - 10 =:> bv.max
  
  //behavior
  FSM behavior {
    State idle {
      Int cur_pos (0)
      Double ratio (0)
      #323232 =: fg.value
      handle.cx - 10 =: cur_pos
      cur_pos == 0 ? 0 : (slider.width - 20)/cur_pos =: ratio
      ratio == 0 ? 10 : (slider.width - 20)/ratio + 10 => handle.cx
      value->{((value / 100) * slider.width + 10*coeff)/coeff =: handle.cx}
    }
    State moving {
      Double offset (0)
      #535353 =: fg.value
      (((handle.cx - 10)*coeff)/slider.width) * 100 =:> value
      handle.press.x - handle.cx =: offset
      handle.move.x - offset =:> bv.input
      bv.result =:> handle.cx
    }
    idle->moving (handle.press)
    moving->idle (handle.release)
  }
}
