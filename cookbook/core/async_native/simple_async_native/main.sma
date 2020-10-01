/*
* djnn Smala compiler
*
* The copyright holders for the contents of this file are:
* Ecole Nationale de l'Aviation Civile, France (2020)
* See file "license.terms" for the rights and conditions
* defined by copyright holders.
*
*
* Contributors:
* Mathieu Magnaudet <mathieu.magnaudet@enac.fr>
* Mathieu Poirier <mathieu.poirier@enac.fr>
*
*/
use core
use base
use display
use gui
import gui.widgets.Button
import gui.interactors.SimpleDrag

 _native_code_
 %{
    // to use C sleep function in smala
    #include <unistd.h>

    // to use exclusive_access in smala
    #include "exec_env/global_mutex.h"
 %}

_action_
smala_action(Process src, Process root)
{
    // blocking call
    sleep (2)

    // then if you have to modify smala/djnn tree
    get_exclusive_access ("")
    root.t2.text = "TEXT FROM ACTION"
    release_exclusive_access ("")

    // another blocking call
    sleep (2)
}

_main_
Component root {
Frame f ("f", 0, 0, 500, 600)
Exit ex (0, 1)
f.close -> ex
FillColor fcc (#000000)
Text explanation1 (10, 20, "Click the button to launch the async action")
Text explanation2 (10, 40, "then, drag the rectangle to check that the application is not freezed")
Text explanation3 (10, 60, "When the action is terminated, \"end\" should appear")
Text t (10, 120, "  ")
Text t2 (10, 140, "")
Button btn (f, "launch", 50, 150)
FillColor fc (#FF00FF)
Rectangle r (200, 200, 100, 100, 0, 0)
Ref toDrag (r)
SimpleDrag _ (toDrag, f)

// Bind a C++ native action
NativeAsyncAction cpp_na (smala_action, root, 1)
btn.click -> cpp_na
btn.click -> {"STARTED" =: t.text}
cpp_na.end->{"END" =: t.text}
}
