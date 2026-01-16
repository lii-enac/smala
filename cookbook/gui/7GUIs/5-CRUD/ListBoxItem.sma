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
ListBoxItem (Process _model, int _index) inherits IWidget () {
  model aka _model
  Int index (_index)

  Int LBI_HEIGHT (18)
  Int LBI_HZ_MARGIN (6)

  // Int text_color (#FFFFFF)

  Spike select
  Spike unselect

  Component bg {
    //FillOpacity _ (0.5)
    FillColor fill_c (#DDDDDD)
    OutlineWidth _ (1)
    OutlineColor outln_c (#666666)

    Rectangle r (0, 0, 0, $LBI_HEIGHT, 2, 2)
    // this.min_width =:> r.width
    // this.preferred_width =:> r.width
  }

  Translation offset (0, 0)

  Component label {
    FillColor fill_c (#000000)

    Text txt ($LBI_HZ_MARGIN / 2, 0, "item " + toString (index))

  }
  
  text aka label.txt.text
  LBI_HEIGHT / 2 + label.txt.ascent / 2 - 1 =:> offset.ty

  label.txt.width + LBI_HZ_MARGIN =: this.min_width
  this.preferred_width == -1 ? label.txt.width + LBI_HZ_MARGIN : this.preferred_width =: this.preferred_width, bg.r.width

  LBI_HEIGHT =: this.min_height, this.preferred_height

  FSM fsm_selection {
    State st_unselected {
      #DDDDDD =: bg.fill_c.value
      #000000 =: label.fill_c.value
    }

    // State st_hover {

    // }

    State st_selected {
      #197EFF =: bg.fill_c.value
      #FFFFFF =: label.fill_c.value
    }

    st_unselected -> st_selected (bg.r.press, select)
    st_selected -> st_unselected (unselect)

    // (bg.r.enter)
    // (bg.r.leave)
  }
}