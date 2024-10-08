/*
*  Multitouch drag
*
*  The copyright holders for the contents of this file are:
*  Ecole Nationale de l'Aviation Civile, France (2017-2024)
*  See file "license.terms" for the rights and conditions
*  defined by copyright holders.
*
*
*  Contributors:
*     Mathieu Magnaudet <mathieu.magnaudet@enac.fr>
*  		Stephane Conversy <stephane.conversy@enac.fr>
*     Mathieu poirier <mathieu.poirier@enac.fr>
*
*/

use core
use base
use display
use gui

/* ----- howto deactivate Gesture recogintion
*  Linux:
*  - wget "https://extensions.gnome.org/extension-data/disable-gestures-2021verycrazydog.gmail.com.v4.shell-extension.zip"
*  - gnome-extensions install disable-gestures-2021verycrazydog.gmail.com.v4.shell-extension.zip
*  - Log out of your session and log back in
*  - (sometimes it also needs : gnome-extensions enable disable-gestures-2021@verycrazydog.gmail.com)
*/

_main_
Component root
{
  //activate touches
  _ENABLE_TOUCHES = 1

	Frame f ("Multitouch rrr", 0, 0, 800, 1000)
  Exit ex (0, 1)
  f.close -> ex
	
  OutlineColor _(0,255,255)

  Component rrr_rectangle1 {
    Homography h (1, 0, 0, 400, 0, 1, 0, 400, 0, 0, 1, 0, 0, 0, 0, 1)
    FillColor _ (255, 0, 0)
    Rectangle r (0, 0, 150, 150, 0, 0)
    FillColor _ (0, 0, 0)
    Text _ (10, 20, "2T")
  }
  RRR_2T _ (rrr_rectangle1.r, rrr_rectangle1.h)

  Component rrr_rectangle2 {
    Homography h (1, 0, 0, 200, 0, 1, 0, 200, 0, 0, 1, 0, 0, 0, 0, 1)
    FillColor _ (0, 255, 0)
    Rectangle r (0, 0, 150, 150, 0, 0)
    FillColor _ (0, 0, 0)
    Text _ (10, 20, "MT")
  }
  
  RRR_MT _ (rrr_rectangle2.r, rrr_rectangle2.h)
}