/*
*  djnn Smala compiler
*
*  The copyright holders for the contents of this file are:
*    Ecole Nationale de l'Aviation Civile, France (2017)
*  See file "license.terms" for the rights and conditions
*  defined by copyright holders.
*
*
*  Contributors:
*    
*     Mathieu Magnaudet <mathieu.magnaudet@enac.fr>
*     Stephane Conversy <stephane.conversy@enac.fr>
*     Mathieu Poirier <mathieu.poirier@enac.fr>
*/

use core
use utils
use base
use display
use gui

_action_
cpp_action (Process c)
%{
  //cerr << "coucou" << endl;
%}

_main_
Component root {
    Frame f ("simplest", 0, 0, 600, 600)

    Clock cl (1000)

    // Bind a C++ native action
    NativeAction cpp_na (cpp_action, root, 1)
    cl.tick -> cpp_na

    // GenericMouse.right.press -> (root) {
    //     display_creation_stats ()
    // }

}
