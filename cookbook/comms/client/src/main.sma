/*
*  Simple RemoteProc app
*
*  The copyright holders for the contents of this file are:
*  Ecole Nationale de l'Aviation Civile, France (2017-2020)
*  See file "license.terms" for the rights and conditions
*  defined by copyright holders.
*
*  Contributors:
*    Mathieu Magnaudet   <mathieu.magnaudet@enac.fr>
* 
*/

use core
use base
use display
use gui
use comms

_main_
Component root {
  Frame f ("RemoteProc - client", 100, 100, 400, 200)
  Exit ex (0, 1)
  f.close -> ex
  FillColor _ (#101010)
  NoOutline _
  Rectangle bkg (0, 0, 0, 0, 0, 0)
  f.{width, height} =:> bkg.{width, height}

  Component expl {
    FillColor _ (White)
    Text t (10, 15, "If the circle is red, start the server then click on the circle")
    Text t2 (10, 30, "to enable connection.")
    Text t3 (10, 45, "Click on the rectangle to increment the server's counter.")
  }
  Translation _ (0, 45)

  RemoteProc dp ("127.0.0.1", 8080)

  Component sub {
    FillColor fc (#FF0000)
    Circle c (50, 50, 20)
    c.press->dp.connect
    Switch set_col (false) {
      Component true {
       #10FF10 =: fc.value
      }
      Component false {
        #FF0000 =: fc.value
      }
    }
    dp.status =:> set_col.state
  }

  Incr inc (1)
  FillColor _ (White)
  Text tp (50, 100, "")

  FillColor _ (100, 100, 250)
  Rectangle r (100, 30, 40, 40, 5, 5)
  r.press -> dp.inc
  dp.inc.state =:> tp.text
}