/*
 *	djnn Smala compiler
 *
 *	The copyright holders for the contents of this file are:
 *		Ecole Nationale de l'Aviation Civile, France (2020)
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

import gui.widgets.Button

_action_
list_action (list l, Process c)
%{
  Process *data = (Process*) get_native_user_data (c);
  for (auto e: l) {
    double w = ((AbstractProperty*)e->find_child("width"))->get_double_value ();
    ((DoubleProperty*)e->find_child("width"))->set_value (w - 5, true);
  }
%}


_main_
Component root {

	Frame f ("f", 0, 0, 500, 600)
    Exit ex (0, 1)
    f.close -> ex
    NoOutline _
    FillColor _ (10, 10, 10)
    Rectangle bkg (0, 0, 0, 0, 0, 0)
    f.width =:> bkg.width
    f.height =:> bkg.height
	FillColor fcc (#FFFFFF)
    Text explanation1 (10, 20, "Select rectangles by clicking on them")
    Text explanation2 (10, 40, "Click on the Action button to resize them")
    Text explanation3 (10, 60, "Unselect them by clicking on the Clear button")

	FillColor _ (#FF0000)
	Rectangle red (50, 400, 100, 100, 0, 0)
	FillColor _ (#00FF00)
	Rectangle green (200, 400, 100, 100, 0, 0)
	FillColor _ (#0000FF)
	Rectangle blue (350, 400, 100, 100, 0, 0)
    FillColor _ (#000000)
   
    Button act (f, "Action", 200, 150)
    Button clear (f, "Clear", 300, 150)

    ProcessCollector collection
    red.press->{red =: collection.add}
    green.press->{green =: collection.add}
    blue.press->{blue =: collection.add}
    clear.click->collection.rm_all

    NativeCollectionAction coll_act (list_action, collection, 1)
    act.click->coll_act
}

