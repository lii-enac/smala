/*
*  Smala cookbook stylus_and_touches
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
    Translation tr (0, 0)

    Circle finger (0, 0, 50)

    touch.move.x =:> tr.tx // finger.cx
    touch.move.y =:> tr.ty // finger.cy

    FillColor _ (#FFFFFF)
    Text t (-20, -70, toString(touch.id))
}
