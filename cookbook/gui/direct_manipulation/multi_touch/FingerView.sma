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
*     Vincent Peyruqueou <vincent.peyruqueou@enac.fr>
*
*/
use core
use base
use gui

_define_
FingerView (Process touch) {
    Circle finger (-100, -100, 100)
    touch.move.x =:> finger.cx
    touch.move.y =:> finger.cy
}
