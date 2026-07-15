/*
*  djnn Smala compiler
*
*  The copyright holders for the contents of this file are:
*    Ecole Nationale de l'Aviation Civile, France (2023)
*  See file "license.terms" for the rights and conditions
*  defined by copyright holders.
*
*
*  Contributors:
*     Mathieu Magnaudet <mathieu.magnaudet@enac.fr>
*     Mathieu Poirier <mathieu.poirier@enac.fr>
*
*/
use core 
use base
use gui

import Chat

_main_
Component root {
    Frame f ("Chat", 500, 0, 210, 310)
    Exit ex (0, 1)
    f.close->ex
    mouseTracking = 1 // for send button :-/

    f.background_color.r = 240
    f.background_color.g = 240
    f.background_color.b = 240

    Chat chat (0, 0, 200, 300, f)
    //f.{width, height} =:> chat.{width, height}

    // _DEBUG_GRAPH_CYCLE_DETECT = 1
    // _DEBUG_SEE_ACTIVATION_SEQUENCE = 1
    // _DEBUG_SEE_ACTIVATION_SEQUENCE_TARGET_LOCATION = "paging.sma:56"
}