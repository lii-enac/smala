/*
*  djnn Smala compiler
*
*  The copyright holders for the contents of this file are:
*    Ecole Nationale de l'Aviation Civile, France (2024)
*  See file "license.terms" for the rights and conditions
*  defined by copyright holders.
*
*
*  Contributors:
*    
*     Mathieu Poirier <mathieu.poirier@enac.fr>
*/

use core
use base
use gui

_define_
View(int _ty) {
  Translation pos (10, _ty)
  FillColor _ (Green)
  Circle _ (0, -5, 5)
  FillColor _ (White)
  String info1 ("---")
  String info2 ("---")
  String info3 ("---")
  String info4 ("---")
  String info5 ("---")
  String info6 ("---")
  String info7 ("---")
  Text line (10, 0, "***")
  info1 + "\t" + info2 + "\t" + info3 + "\t" + info4 + "\t" + info5 + "\t" + info6 + "\t" + info7 =:> line.text
}