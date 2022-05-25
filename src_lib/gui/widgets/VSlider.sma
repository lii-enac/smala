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

import gui.widgets.IWidget

_define_
VSlider (int _init_val) inherits IWidget () {
  BoundedValue bv (0, 100, _init_val)
  min aka bv.min
  max aka bv.max
  output aka bv.result
  Double value (_init_val)
  bv.result => value
  int radius = 8

  OutlineColor _ (#535353)
  FillColor bg_color (White)
  Rectangle bg (7, 0, 6, 140, 3, 3)
  FillColor fill_color (#959595)
  Rectangle fill (7, 0, 6, 0, 3, 3)
  OutlineColor _ (#535353)
  OutlineWidth handle_width (1)
  FillColor handle_color (#959595)
  Translation pos (0, 0)
  Circle handle (10, 0, radius)

  this.height - 2*radius  =:> bg.height

  this.min_width = 2*radius
  this.min_height = 100
  this.preferred_width = 2*radius

  BoundedValue bv_pos (0, $this.height, $pos.ty)
  bg.height =:> bv_pos.max

  FSM fsm_handle {
    State idle {
      #959595 =: handle_color.value
      (value-min)/(max-min) * (bg.height) =:> pos.ty, fill.height
    }
    State hover {
      #FFFFFF =: handle_color.value
    }
    State move {
      Double off_y (0)
      handle.press.y - pos.ty =: off_y
      handle.move.y - off_y =:> bv_pos.input
      bv_pos.result => pos.ty, fill.height
      (pos.ty/(bg.height)) * (max-min) + min =:> bv.input
    }
    idle->hover (handle.enter)
    hover->idle (handle.leave)
    hover->move (handle.press)
    move->idle (handle.release)
  }
}
