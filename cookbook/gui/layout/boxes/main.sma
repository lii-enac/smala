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

import gui.widgets.WidgetContainer
import gui.widgets.PushButton
import gui.widgets.ToggleButton
import gui.widgets.HBox
import gui.widgets.VBox
import gui.widgets.Slider
import gui.widgets.Label
import gui.widgets.ComboBox

_main_
Component root {
  Frame f ("frame", 0, 0, 600, 200)
  Exit ex (0, 1)
  f.close->ex
  FillColor _ (200, 200, 200)
  NoOutline _
  Rectangle bkg (0, 0, 0, 0, 0, 0)
  f.{width,height}=:>bkg.{width,height}
  TextPrinter tp

  WidgetContainer wc1 (f) {
    HBox hbox (0)
    WidgetContainer sub_wc (wc1) {
      VBox vbox (1)
      ComboBox cb (wc1, 0, 0, 0, 0)
      addChildrenTo cb.str_items {
        String _ ("First")
        String _ ("Second")
        String _ ("Third")
      }
      PushButton b1 (wc1, "My Button", 0, 0)
      addChildrenTo vbox.items
      { 
        cb,
        b1
      }
    }
    PushButton b2 (wc1, "Quit", 0, 0)
    b2.click->ex
    PushButton b3 (wc1, "Yet Another Button", 0, 0)
    b3.click -> { "Hello!" =: tp.input }
    WidgetContainer wc (wc1) {
      VBox vbox (1)
      PushButton sub1 (wc, "Sub button 1", 0, 0)
      PushButton sub2 (wc, "Sub button 2", 0, 0)
      ToggleButton sub3 (wc, "Connect slider", 0, 0)
      Label l (wc, "Value: ", 0, 0)
      Slider s (wc, 0, 0)
      FSM fsm {
        State idle
        State connected {
          "Value: " + s.value =:> l.text
        }
        idle->connected (sub3.toggle)
        connected->idle (sub3.toggle)
      }      
      addChildrenTo vbox.items
      { 
        sub1,
        sub2,
        sub3,
        l,
        s 
      }
    }
    addChildrenTo hbox.items { sub_wc, wc, b2, b3 }
  }
}
