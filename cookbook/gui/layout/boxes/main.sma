/*
 *	djnn Smala compiler
 *
 *	The copyright holders for the contents of this file are:
 *		Ecole Nationale de l'Aviation Civile, France (2021)
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
use gui

import gui.widgets.PushButton
import gui.widgets.ToggleButton
import gui.widgets.HBox
import gui.widgets.VBox
import gui.widgets.HSlider
import gui.widgets.Label
import gui.widgets.ComboBox
import gui.widgets.HSpace

_main_
Component root {
  Frame f ("frame", 0, 0, 600, 200)
  Exit ex (0, 1)
  f.close->ex
  mouseTracking = 1 
  FillColor _ (200, 200, 200)
  NoOutline _
  Rectangle bkg (0, 0, 0, 0, 0, 0)
  f.{width,height}=:>bkg.{width,height}
  TextPrinter tp

  HBox hbox (f)
    VBox vbox (hbox)
      ComboBox cb
      addChildrenTo cb.str_items {
        String _ ("First choice")
        String _ ("Second choice")
        String _ ("Third choice")
      }
      PushButton b1 ("My Button")
      cb.value =:> b1.label
      addChildrenTo vbox.items
      { 
        cb,
        b1
      }
    HSpace hspace (20)
    PushButton b2 ("Quit")
    b2.click->ex
    PushButton b3 ("Say Hello!")
    b3.click -> { "Hello!" =: tp.input }
    VBox vbox2 (hbox)
      PushButton sub1 ("Sub button 1")
      PushButton sub2 ("Sub button 2")
      ToggleButton sub3 ("Connect slider")
      Label l ("Value: ")
      l.h_alignment = 0
      HSlider s (0)
      FSM fsm {
        State idle
        State connected {
          "Value: " + s.value =:> l.text
        }
        idle->connected (sub3.toggle)
        connected->idle (sub3.toggle)
      }
      addChildrenTo vbox2.items
      { 
        sub1,
        sub2,
        sub3,
        l,
        s 
      }
  addChildrenTo hbox.items { vbox, hspace, vbox2, b3, b2 }
 }
