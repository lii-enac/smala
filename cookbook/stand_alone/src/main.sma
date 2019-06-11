/*
 *	djnn Smala compiler
 *
 *	The copyright holders for the contents of this file are:
 *		Ecole Nationale de l'Aviation Civile, France (2017)
 *	See file "license.terms" for the rights and conditions
 *	defined by copyright holders.
 *
 *
 *	Contributors:
 *		Mathieu Magnaudet <mathieu.magnaudet@enac.fr>
 *
 */

use core
use base
use display
use gui

import Button


_action_
myFunc (Process c) 
%{
  cout << ("Button clicked") << endl;
%}


_main_
Component root {
  Frame f ("my frame", 0, 0, 400, 600)
  Exit ex (0, 1)
  f.close -> ex

  NativeAction na (myFunc, 1)


  Button b (f, "myButton", 10, 100)
  b.click -> na
}

run root
run syshook
