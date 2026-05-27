/*
 *	djnn Smala compiler
 *
 *	The copyright holders for the contents of this file are:
 *		Ecole Nationale de l'Aviation Civile, France (2017-2026)
 *	See file "license.terms" for the rights and conditions
 *	defined by copyright holders.
 *
 *
 *	Contributors:
 *		Vincent Peyruqueou <vincent.peyruqueou@enac.fr>
 *
 */
use core
use base
use display
use gui

import gui.widgets.IWidget

_define_
// ListBoxItem (int _index) inherits IWidget ()

// - _horizontal_margin: Horizontal margin on the left & on the right of the label
// - _height: height of the list box item
ListBoxItem (int _horizontal_margin, int _height) inherits IWidget ()
{
  // Int index (_index)
  Bool _is_binded (false)
  Bool is_selected (false)

  Int HORIZONTAL_MARGIN (_horizontal_margin)  // Horizontal margin on the left & on the right of the label

  this.h_alignment = 0  // List box item are aligned to the left
  
  // _height =: this.min_height, this.preferred_height  // FIXME: does NOT work
  this.min_height = _height
  this.preferred_height = _height

  // Int text_color (#FFFFFF)

  Spike select
  Spike unselect

  Component bg {
    FillOpacity fill_op (0.5)
    FillColor fill_c (#DDDDDD)

    NoOutline _
    // OutlineWidth _ (1)
    // OutlineColor outln_c (#666666)

    Rectangle r (0, 0, 0, _height, 0, 0)
  }

  Component label {
    FillColor fill_c (#000000)

    // Default text: "item " + index
    // Text txt (_horizontal_margin, 0, "item " + toString (index))
    Text txt (_horizontal_margin, 0, "item")
    this.min_height / 2 + txt.ascent / 2 - 1 =:> txt.y
  }
  text aka label.txt.text

  label.txt.width + (2 * HORIZONTAL_MARGIN) =:> this.min_width
  (this.preferred_width == -1) || (this.preferred_width < this.min_width) ? this.min_width : this.preferred_width =:> this.preferred_width, bg.r.width

  // _height =: this.min_height, this.preferred_height

  FSM fsm_selection {
    State st_unselected {
      0 =: is_selected

      0.0 =: bg.fill_op.a
      #DDDDDD =: bg.fill_c.value
      #000000 =: label.fill_c.value
    }

    State st_selected {
      1 =: is_selected

      1.0 =: bg.fill_op.a
      #197EFF =: bg.fill_c.value
      #FFFFFF =: label.fill_c.value
    }

    st_unselected -> st_selected (bg.r.press, select)
    st_selected -> st_unselected (unselect)
  }

  // FSM fsm_hover {
  //   out -> in (bg.r.enter)
  //   in -> out (bg.r.leave)
  // }
  
}