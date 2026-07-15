/*
*  Smala cookbook boxes
*
*  The copyright holders for the contents of this file are:
*    Ecole Nationale de l'Aviation Civile, France (2021-2026)
*  See file "license.terms" for the rights and conditions
*  defined by copyright holders.
*
*
*  Contributors:
*     Mathieu Magnaudet <mathieu.magnaudet@enac.fr>
*     Mathieu Poirier <mathieu.poirier@enac.fr>
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
import gui.widgets.UITextField

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

  HBox hbox {
    VBox vbox {
      Component model {
        List items {
          String _ ("First choice")
          String _ ("Second choice")
          String _ ("Third choice")
        }
      }
      PushButton b1 ("My Button")
      b1.preferred_width = 100
      
      UITextField tf
      tf.preferred_width = 100

      ComboBox cb
      model =: cb.model
      cb.preferred_width = 100
      cb.value =:> b1.label
    }
    HSpace hspace (20)
    PushButton b2 ("Quit")
    b2.click->ex
    PushButton b3 ("Say Hello!")
    b3.click -> { "Hello!" =: tp.input }
    VBox vbox2 {
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
    }
  }
 }
