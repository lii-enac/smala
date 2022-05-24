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

import gui.widgets.IWidget

_define_
Slider (int _init_val) inherits IWidget () {
  BoundedValue bv (0, 100, _init_val)
  min aka bv.min
  max aka bv.max
  output aka bv.result
  Double value (_init_val)
  bv.result => value

  Translation _ (10, 0)
  //NoOutline _
  OutlineColor _ (200, 200, 200)
  FillColor bg_color (White)
  Rectangle bg (0, 7, 140, 6, 3, 3)
  FillColor fill_color (#535353)
  Rectangle fill (0, 7, 0, 6, 3, 3)
  FillColor handle_color (#323232)
  Translation pos (0, 0)
  Circle handle (0, 10, 10)

  this.width - 20  =:> bg.width

  this.min_width = 100
  this.min_height = 20
  this.preferred_height = 20

  BoundedValue bv_pos (0, $this.width, $pos.tx)
  bg.width =:> bv_pos.max

  FSM fsm_handle {
    State idle {
      #323232 =: handle_color.value
      (value-min)/(max-min) * (bg.width) =:> pos.tx, fill.width
    }
    State move {
      #535353 =: handle_color.value
      Double off_x (0)
      handle.press.x - pos.tx =: off_x
      handle.move.x - off_x =:> bv_pos.input
      bv_pos.result => pos.tx, fill.width 
      (pos.tx/(bg.width)) * (max-min) + min =:> bv.input
    }
    idle->move (handle.press)
    move->idle (handle.release)
  }
}
