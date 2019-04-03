/*
 *	djnn Smala compiler
 *
 *	The copyright holders for the contents of this file are:
 *		Ecole Nationale de l'Aviation Civile, France (2019)
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

_main_
Component root {
  Frame f ("my frame", 0, 0, 1200, 600)
  Button display (f, "Show", 50, 50)
  Button hide (f, "Hide", 50, 150)

  FillColor _(100, 210, 100)
  Rectangle test (50, 250, 100, 70, 0, 0)
  display.click -> test
  hide.click ->! test

  FillColor _ (210, 100, 100)
  Rectangle test2 (250, 250, 100, 70, 0, 0)
  test !-> test2
  test ->! test2

  FillColor _ (100, 100, 210)
  Rectangle test3 (450, 250, 100, 70, 0, 0)
  test !->! test3
  display.click -> test3
}

run root
stop root.test2
run syshook