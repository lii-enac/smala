/*
*  djnn Smala compiler
*
*  The copyright holders for the contents of this file are:
*    Ecole Nationale de l'Aviation Civile, France (2020)
*  See file "license.terms" for the rights and conditions
*  defined by copyright holders.
*
*
*  Contributors:
*    
*     Mathieu Poirier <mathieu.poirier@enac.fr>
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
  
  Text reg_txt (10, 30, "str sent:\t")
  Text foo_txt (10, 50, "foo value:\t")
  Text bar_txt (10, 70, "bar value:\t")

  FillColor _ (0, 255, 0)
  Text reg_value (80, 30, "")
  Text foo_value (80, 50, "")
  Text bar_value (80, 70, "")

  regex.1 => foo_value.text
  regex.2 => bar_value.text

  Incr i1 (0)
  Incr i2 (0)
  i2.delta = 100
  DoubleFormatter df1 (0, 2)
  DoubleFormatter df2 (0, 0)
  i1.state => df1.input
  i2.state => df2.input

  AssignmentSequence assign (1) {

    "foo=" + to_string(df1.output) +  " bar=" + to_string(df2.output) =: reg_value.text
    reg_value.text =: regex.input
  }

  Clock cl (500)
  cl.tick -> i1
  cl.tick -> i2
  cl.tick -> assign
}