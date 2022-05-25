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
HSlider (int _init_val) inherits IWidget () {
  BoundedValue bv (0, 100, _init_val)
  min aka bv.min
  max aka bv.max
  output aka bv.result
  Double value (_init_val)
  bv.result => value
  int radius = 8

  Translation _ (radius, 0)

  OutlineColor _ (#535353)
  FillColor bg_color (White)
  Rectangle bg (0, 7, 140, 6, 3, 3)
  FillColor fill_color (#959595)
  Rectangle fill (0, 7, 0, 6, 3, 3)
  OutlineColor _ (#535353)
  FillColor handle_color (#959595)
  Translation pos (0, 0)
  Circle handle (0, 10, radius)

  this.width - 2*radius  =:> bg.width

  this.min_width = 100
  this.min_height = 2*radius
  this.preferred_height = 2*radius

  BoundedValue bv_pos (0, $this.width, $pos.tx)
  bg.width =:> bv_pos.max

  FSM fsm_handle {
    State idle {
      #959595 =: handle_color.value
      (value-min)/(max-min) * (bg.width) =:> pos.tx, fill.width
    }
    State hover {
      #FFFFFF =: handle_color.value
    }
    State move {
      Double off_x (0)
      handle.press.x - pos.tx =: off_x
      handle.move.x - off_x =:> bv_pos.input
      bv_pos.result => pos.tx, fill.width 
      (pos.tx/(bg.width)) * (max-min) + min =:> bv.input
    }
    idle->hover (handle.enter)
    hover->idle (handle.leave)
    idle->move (handle.press)
    move->idle (handle.release)
  }
}
