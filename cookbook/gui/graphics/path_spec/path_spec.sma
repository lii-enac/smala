/*
*  djnn Smala compiler
*
*  The copyright holders for the contents of this file are:
*    Ecole Nationale de l'Aviation Civile, France (2023-2024)
*  See file "license.terms" for the rights and conditions
*  defined by copyright holders.
*
*
*  Contributors:
*     Mathieu Magnaudet <mathieu.magnaudet@enac.fr>
*     Vincent Peyruqueou <vincent.peyruqueou@enac.fr>
*
*/
use core
use base
use display
use gui

import gui.widgets.StandAlonePushButton
import gui.widgets.StandAloneToggleButton

_main_
Component root {
  Frame f ("paths spec", 0, 0, 1000, 600)
  
  Exit ex (0, 1)
  f.close -> ex

  Bool is_visible_path2 (false)

  String spec1 ("M 100,200 C 100,100 200,500 300,400")
  String spec2 ("M 867.0,256.0 945.98376465,385.31274414 798.5927124,350.15460205 700.01623535,465.23242188 687.9072876,314.19073486 548.0,256.0 687.9072876,197.80926514 700.01623535,46.76756668 798.5927124,161.84539795 945.98376465,126.68724823")
  String spec3 ("m 157.66126,20.571361 5.59732,8.85735 5.59915,-8.85735 z")
  String spec4 ("m 169.02539,39.556641 c 0.56445,-0.250509 1.08339,-0.5873 1.51367,\
      -1.017579 0.40638,-0.40638 0.71379,-0.902493 0.96094,-1.429687 V 12.734996 c -0.24715,\
      -0.527194 -0.55456,-1.023307 -0.96094,-1.429687 -0.39807,-0.398075 -0.88601,-0.695739 -1.40039,-0.941407 h -14.30078 v 29.192739 z")

  StandAlonePushButton btn_spec1 ("Spec 1", 10, 10)
  StandAlonePushButton btn_spec2 ("Spec 2", 10, 50)
  StandAlonePushButton btn_spec3 ("Spec 3", 10, 90)
  //StandAlonePushButton btn_spec4 ("Spec 4", 10, 130)
  btn_spec1.idle_color = #666666
  btn_spec2.idle_color = #666666
  btn_spec3.idle_color = #666666
  //btn_spec4.idle_color = #666666

  StandAloneToggleButton tgb_spec4 ("", 10, 130)
  tgb_spec4.idle_color = #666666

  is_visible_path2 ? "visible" : "hidden" =:> tgb_spec4.label
  tgb_spec4.toggle -> {
      !is_visible_path2 =: is_visible_path2
    }

  TextPrinter tp

  OutlineWidth ow (2)
  FillColor fc (255, 0, 0)
  OutlineColor oc (255, 255, 255)

  Switch switch (false) {
    Component false

    Component true {
      Path path2 (getString (spec4))
    }
  }
  is_visible_path2 =:> switch.state

  Path path (getString (spec1))

  "Spec = " + path.spec =:> tp.input

  btn_spec1.click -> {
    spec1 =: path.spec
  }
  btn_spec2.click -> {
    spec2 =: path.spec
  }
  btn_spec3.click -> {
    spec3 =: path.spec
  }
  // btn_spec4.click -> {
  //   spec4 =: path.spec
  // }

}
