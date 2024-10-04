/*
*  Multitouch touch app
*
*  The copyright holders for the contents of this file are:
*  Ecole Nationale de l'Aviation Civile, France (2018-2024)
*  See file "license.terms" for the rights and conditions
*  defined by copyright holders.
*
*
*  Contributors:
*    Sébastien Leriche <sebastien.leriche@enac.fr>
*    Mathieu Magnaudet <mathieu.magnaudet@enac.fr>
*    Mathieu Poirier <mathieu.poirier@enac.fr>
*    Vincent Peyruqeou <vincent.peyruqueou@enac.fr>
*
*/

use core
use base
use display
use gui

import FingerView

/* ----- howto deactivate Gesture recogintion
*  Linux:
*  - wget "https://extensions.gnome.org/extension-data/disable-gestures-2021verycrazydog.gmail.com.v4.shell-extension.zip"
*  - gnome-extensions install disable-gestures-2021verycrazydog.gmail.com.v4.shell-extension.zip
*  - Log out of your session and log back in
*/

_main_
Component root
{
  //activate touches
  _ENABLE_TOUCHES = 1

  Frame f ("multi_touch", 0, 0, 900, 1000)
  Exit ex (0, 1)
  f.close -> ex
  NoFill _
  OutlineColor _ (100,100,255)
  OutlineWidth _ (10)
  OutlineOpacity _ (0.5)

  // The trick here is to store the views in a component, naming them
  // after the unique ID of the touch they are associated with. 
  // We can then implicitly use find/find_optional (without error returns)
  // to leverage the component's internal map and retrieve the 
  // view corresponding to a touch.
  Component touch_views_map

  f.touches.$added->(root) {
    t = getRef (&root.f.touches.$added)
    Process newview = FingerView (root.touch_views_map, toString(t.id) ,t)
    // print ("\n---- added touch")
    // dump t.id
  }
  f.touches.$removed->set_release_touch:(root) {
    t = getRef (&root.f.touches.$removed)
    // print ("\n--- removed touch")
    // dump t.id
    p = find_optional (root.touch_views_map, toString(t.id))
    delete p
  }
}
