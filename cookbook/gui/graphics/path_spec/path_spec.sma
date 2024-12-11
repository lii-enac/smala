/*
*  Djnn Smala cookbook
*
*  The copyright holders for the contents of this file are:
*    Ecole Nationale de l'Aviation Civile, France (2017-2024)
*  See file "license.terms" for the rights and conditions
*  defined by copyright holders.
*
*
*  Contributors:
*     Vincent Peyruqueou <vincent.peyruqueou@enac.fr>
*     Mathieu Poirier <mathieu.poirier@enac.fr>
*
*/

use core
use base
use display
use gui

import gui.widgets.StandAlonePushButton
//import gui.widgets.StandAloneToggleButton

_main_
Component root {
  Frame f ("paths spec", 0, 0, 1000, 600)
  
  Exit ex (0, 1)
  f.close -> ex

  String spec1 ("M 100,200 C 100,100 200,500 300,400")
  String spec2 ("M 867.0,256.0 945.98376465,385.31274414 798.5927124,350.15460205 700.01623535,465.23242188 687.9072876,314.19073486 548.0,256.0 687.9072876,197.80926514 700.01623535,46.76756668 798.5927124,161.84539795 945.98376465,126.68724823")

  StandAlonePushButton btn_spec1 ("Spec 1", 20, 20)
  StandAlonePushButton btn_spec2 ("Spec 2", 100, 20)
  btn_spec1.idle_color = #666666
  btn_spec2.idle_color = #666666

  TextPrinter tp

  OutlineWidth ow (4)
  FillColor fc (255,0,0)
  OutlineColor _ (0,0,255)

  Path path (getString (spec1))

  "Spec = " + path.spec =:> tp.input

  btn_spec1.click -> {
    spec1 =: path.spec
  }
  btn_spec2.click -> {
    spec2 =: path.spec
  }

}
