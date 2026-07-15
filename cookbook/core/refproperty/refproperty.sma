/*
*  Smala cookbook refproperty
*
*  The copyright holders for the contents of this file are:
*    Ecole Nationale de l'Aviation Civile, France (2019-2025)
*  See file "license.terms" for the rights and conditions
*  defined by copyright holders.
*
*
*  Contributors:
*     Magnaudet Mathieu <mathieu.magnaudet@enac.fr>
*     Vincent Peyruqueou <vincent.peyruqueou@enac.fr>
*
*/
use core
use base
use display
use gui

_main_
Component root
{
  Frame f ("my frame", 0, 0, 500, 300)
  Exit ex (0, 1)
  f.close -> ex

  TextPrinter tp
  TextPrinter tp2

  FillColor _ (100, 100, 100)
  Rectangle r1 (10, 30, 150, 70, 0, 0)
  Rectangle r2 (250, 30, 150, 70, 0, 0)

  FillColor _ (150, 200, 150)
  Rectangle r3 (10, 140, 80, 40, 3, 3)
  Rectangle r4 (160, 140, 80, 40, 3, 3)

  FillColor _ (200, 150, 150)
  Rectangle r5 (10, 250, 80, 40, 3, 3)
  
  RefProperty current (r1)
  // RefProperty current (null)
  
  AssignmentSequence set_r1 (1) {
    r1 =: current
    // "set r1 as current" =: tp.input
  }
  AssignmentSequence set_r2 (1) {
    r2 =: current
    // "set r2 as current" =: tp.input
  }

  Deref deref (current, "press")

  AssignmentSequence set_press (1) {
    "press" =: deref.path
  }
  AssignmentSequence set_move (1) {
    "move" =: deref.path
  }
  
  Incr incr (1)
  deref.activation -> incr

  r1.press -> set_r1
  r2.press -> set_r2
  r3.press -> set_move
  r4.press -> set_press

  r5.press -> na_set_null:(root) {
    setRef (root.current, null)
  }
  // r5.press -> {
  //   nullptr =: current
  // }

  FillColor _ (White)
  Text _ (10, 12, "Click alternately two times on the left and right grey rectangles,")
  Text _ (10, 26, "then click on the red one and click and move on the grey ones")

  Text t (10, 200, "")
  incr.state =:> t.text

  TextAnchor _ (DJN_MIDDLE_ANCHOR)
  Text _ ($r1.x + $r1.width / 2, $r1.y + $r1.height / 2, "set rect 1")
  Text _ ($r2.x + $r2.width / 2, $r2.y + $r2.height / 2, "set rect 2")
  Text _ ($r3.x + $r3.width / 2, $r3.y + $r3.height / 2, "'.move'")
  Text _ ($r4.x + $r4.width / 2, $r4.y + $r4.height / 2, "'.press'")
  Text _ ($r5.x + $r5.width / 2, $r5.y + $r5.height / 2, "set null")

  "Ref current is null ? " + current.is_null =:> tp.input

  current.is_null.false -> {
    "Ref current is NOT null" =: tp2.input
  }

  current.is_null.true -> {
    "Ref current is NULL" =: tp2.input
  }
}

