/*
*  djnn Smala compiler
*
*  The copyright holders for the contents of this file are:
*    Ecole Nationale de l'Aviation Civile, France (2020-2026)
*  See file "license.terms" for the rights and conditions
*  defined by copyright holders.
*
*
*  Contributors:
*     Mathieu Poirier <mathieu.poirier@enac.fr>
*     Magnaudet Mathieu <mathieu.magnaudet@enac.fr>
*     Vincent Peyruqueou <vincent.peyruqueou@enac.fr>
*
*/
use core
use comms
use base
use gui

_main_
Component root {

  Frame f ("regex", 0, 0, 600, 600)
  Exit ex (0, 1)
  f.close -> ex

  Regex regex ("foo=(\\S*) bar=(\\S*)")
  
  FillColor _ (White)
  Text reg_txt (10, 30, "str sent:")
  Text foo_txt (10, 50, "foo value:")
  Text bar_txt (10, 70, "bar value:")

  FillColor _ (0, 128, 0)
  Text reg_value (80, 30, "")
  Text foo_value (80, 50, "")
  Text bar_value (80, 70, "")

  regex.[1] => foo_value.text
  regex.[2] => bar_value.text

  Incr i1 (0)
  Incr i2 (0)
  i2.delta = 100.01

  AssignmentSequence assign (1) {

    "foo=" + toString(i1.state) +  " bar=" + toString(i2.state) =: reg_value.text
    reg_value.text =: regex.input
  }

  Clock cl (500)
  cl.tick -> i1
  cl.tick -> i2
  cl.tick -> assign
}