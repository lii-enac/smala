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
*     Mathieu Poirier   <mathieu.poirier@enac.fr>
*     Magnaudet Mathieu <mathieu.magnaudet@enac.fr>
*
*/
/*
* note: 
* you can use "ivyprobe" to send a message to this application : smala (.*)
* 
* note: on Mac 127.0.0.1 do not work correctly --> use 224.1.2.3 for test instead 
*/

use core
use base
use display
use gui
use comms

_main_
Component root {
  Frame f ("ProcExporter - server", 500, 100, 200, 200)
  Exit ex (0, 1)
  f.close -> ex
  FillColor _ (#101010)
  NoOutline _
  Rectangle bkg (0, 0, 0, 0, 0, 0)
  f.{width, height} =:> bkg.{width, height}
  ProcExporter pe (root, 8080)
  
  Incr inc (1)
  FillColor _ (White)
  Text t (80, 100, "")
  inc.state =:> t.text

  FillOpacity _ (0.5)
  FillColor _ (200, 100, 100)
  Circle c (50, 50, 20)
}