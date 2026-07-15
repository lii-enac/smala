/*
*  djnn Smala compiler
*
*  The copyright holders for the contents of this file are:
*    Ecole Nationale de l'Aviation Civile, France (2025)
*  See file "license.terms" for the rights and conditions
*  defined by copyright holders.
*
*
*  Contributors:
*     Mathieu Poirier <mathieu.poirier@enac.fr>
*
*/
use core
use base
use display
use gui

import Feedback

_main_
Component root {
  Frame frame ("lazy_connectors", 0, 0, 600, 600)
  frame.close ->! mainloop

  int y_feedback = 230

  Component using_int {
    FillColor _ (White)
    Incr incr (0)
    Int result (0)
    Text tresult (10, 20, "")
    incr.state =:> result
    "On Value: " + result =:> tresult.text 
    
    Component buttons {
      Translation _ (400, 10)
      FillColor _ (White)
      Rectangle b1 (0, 0, 70, 30, 0, 0)
      Rectangle b2 (100, 0, 70, 30, 0, 0)
      FillColor _ (Black)
      Text tplus1 (10, 20, "+1")
      Text tsame (110, 20, "same")

      b1.release -> incr 
      b2.release -> {
        result =: result
      }
    }

    Component connectors_int {
      Translation _ (0, 40)  

      Text tc1 (10, 0, "Connector init:")
      Int vc1 (0)
      "Connector with init:\t=:>  " + vc1 => tc1.text
      result =:> vc1
      Feedback fc1 (y_feedback, -15, 300)
      vc1 -> fc1.trigger

      Text tc2 (10, 20, "Connector NON init: \t=>")
      Int vc2 (0)
      "Connector :\t\t=> " + vc2 => tc2.text
      result => vc2
      Feedback fc2 (y_feedback, 5, 300)
      vc2 -> fc2.trigger

      Text tc3 (10, 40, "Connector ? init: \t=?:>")
      Int vc3 (0)
      "Connector ? with init:\t=?:> " + vc3 => tc3.text
      result =?:> vc3
      Feedback fc3 (y_feedback, 25, 300)
      vc3 -> fc3.trigger

      Text tc4 (10, 60, "Connector ? NON init:\t=?> ")
      Int vc4 (1) // should be init the same as result
      "Connector ? :\t=?> " + vc4 => tc4.text
      result =?> vc4
      Feedback fc4 (y_feedback, 45, 300)
      vc4 -> fc4.trigger

    }
  }

  FillColor _ (White)
  Text _ (0, 120, "--------------------")

  Translation _ (0, 140)
  Component using_string {
    FillColor _ (White)
    String result ("foo")
    Text tresult (10, 20, "")
    "On String: " + result =:> tresult.text 
    
    Component buttons {
      Translation _ (400, 10)
      FillColor _ (White)
      Rectangle b1 (0, 0, 70, 30, 0, 0)
      Rectangle b2 (100, 0, 70, 30, 0, 0)
      FillColor _ (Black)
      Text tfoo (10, 20, "foo")
      Text tbar (110, 20, "bar")

      b1.release -> {
        "foo" =: result
      } 
      b2.release -> {
        "bar" =: result
      }
    }

    Component connectors_string {
      Translation _ (0, 40)  

      Text tc1 (10, 0, "Connector init:")
      String vc1 ("")
      "Connector with init:\t=:> " + vc1 => tc1.text
      // should be init after the connector on tc1.text else it won't react
      result =:> vc1
      Feedback fc1 (y_feedback, -15, 300)
      vc1 -> fc1.trigger

      Text tc2 (10, 20, "Connector NON init:\t=> ")
      String vc2 ("")
      "Connector :\t\t=> " + vc2 => tc2.text
      result => vc2
      Feedback fc2 (y_feedback, 5, 300)
      vc2 -> fc2.trigger

      Text tc3 (10, 40, "Connector ? init: ")
      String vc3 ("")
      "Connector ? with init:\t=?:> " + vc3 => tc3.text
      result =?:> vc3
      Feedback fc3 (y_feedback, 25, 300)
      vc3 -> fc3.trigger

      Text tc4 (10, 60, "Connector ? NON init:\t=?> ")
      String vc4 ("foo") // should be init the same as result
      "Connector ? :\t=?> " + vc4 => tc4.text
      result =?> vc4
      Feedback fc4 (y_feedback, 45, 300)
      vc4 -> fc4.trigger

    }
  }
}
