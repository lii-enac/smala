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
use gui

import Button

%{
void
myFunc (Component c) {
  System.out.println ("this is my c code");
}
%}

%{
public static String
getToto ()
{
	return "toto";
}
%}

_main_
Component root {
  Frame f ("my frame", 0, 0, 400, 600)
  Exit ex (0, 1)
  f.close -> ex

  string toto = CCall (getToto)
  Text myText (300, 100, toto)
  NativeAction na (myFunc, 1)

  myList = clone (n=5) {
    Button b (f, "myButton", 10, n * 100)
    b.click->na
  }

  e = find (myList.1.b)
  e.x = 100.0
  Button b2 (f, "myVeryLongButton", 10, 150)
  b2.click->ex

  f.width > 200 ? 100 : 10 => b2.x
}

run root
run syshook
