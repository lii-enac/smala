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
 *    Mathieu Magnaudet <mathieu.magnaudet@enac.fr>
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
#include <cmath>
%}

// Define a C++ native action to be executed asynchronously
_action_
cpp_action (Process p)
%{
  for (int i = 0; i < 500000000; i++) {
    int candidate = i * (1000000000 * rand ());
    bool isPrime = true;
    for (int c = 2; c <= sqrt(candidate); ++c) {
      if (candidate % c == 0) {
          // not prime
          isPrime = false;
          break;
       }
    }
  }
%}


_main_
Component root {

  Frame f ("f", 0, 0, 500, 600)
  Exit ex (0, 1)
  f.close -> ex
  
  FillColor fcc (#000000)
  Text explanation1 (10, 20, "Click the button to launch the async action")
  Text explanation2 (10, 40, "then, drag the rectangle to check that the application is not freezed")
  Text explanation3 (10, 60, "When the action is terminated, \"end\" should appear")

  Text t (10, 120, "")

  Button btn (f, "launch", 50, 150)
  FillColor fc (#FF00FF)
  Rectangle r (200, 200, 100, 100, 0, 0)
  Ref toDrag (r)
  SimpleDrag _ (toDrag, f)
   


    // Bind a C++ native action
  NativeAsyncAction cpp_na (cpp_action, root, 1)
  btn.click -> cpp_na
  btn.click -> {"start" =: t.text}
  cpp_na.end->{"end" =: t.text}
}

