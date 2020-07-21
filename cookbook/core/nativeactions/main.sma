/*
 *	djnn Smala compiler
 *
 *	The copyright holders for the contents of this file are:
 *		Ecole Nationale de l'Aviation Civile, France (2018)
 *	See file "license.terms" for the rights and conditions
 *	defined by copyright holders.
 *
 *
 *	Contributors:
 *		Mathieu Poirier <mathieu.poirier@enac.fr>
 *
 */

use core
use base
use display
use gui

// Define a C++ native action
_action_
cpp_action (Process c)
%{
	// To get the source that triggered the native action:
	//Process *source = c->get_activation_source ();
	
	// To get the user_data:
 	Process *data = (Process*) get_native_user_data (c);

 	Process *fc = dynamic_cast<Process*>(data->find_child ("fc"));

    ((IntProperty*) fc->find_child ("value"))->set_value (0xFF0000, 1)  ;

    // to print the component tree:
 	//fc->dump(0);
%}

// Define a smala native action
_action_
smala_action (Process src, Process data)
{   // To create a NEW component (either visual or assignment/binding/connector):
    // add it to a parent (passed for exemple as data), thanks to
    //     addChildrenTo parent {
    //         ...the new components...
    //     }

    // To set a value on an EXISTING property:
    // do not use an assignment as it would create a new Assignment component
    // and add it to the component tree each time this native is invoked.
    // The value change is propagated.
    data.value = #00FF00

    // to print the component tree:
    //dump data
}

_action_
list_action (list l, Process c)
%{
  Process *data = (Process*) get_native_user_data (c);
  for (auto e: l) {
    std::cout << "Rectangle x = " << ((AbstractProperty*)e->find_child("x"))->get_double_value () << std::endl;
  }
%}


_main_
Component root {

	Frame f ("f", 0, 0, 500, 600)
    Exit ex (0, 1)
    f.close -> ex
	FillColor fcc (#FFFFFF)
    Text explanation1 (10, 20, "Press color rectangle to give his color to the upper rectangle")
    Text explanation2 (10, 40, "!! Blue rectangle has 3 press zone that will change his y position")
    Text explanation3 (10, 60, "- if blue.press.x < 380 then blue.y = 350 ")
    Text explanation4 (10, 80, "- if 380 < blue.press.x < 420 then blue.y = 450 ")
    Text explanation5 (10, 100, "- if 420 < blue.press.x then blue.y = 400 ")

    FillColor fc (#FFFFFF)
	Rectangle _ (200, 200, 100, 100, 0, 0)
	FillColor _ (#FF0000)
	Rectangle red (50, 400, 100, 100, 0, 0)
	FillColor _ (#00FF00)
	Rectangle green (200, 400, 100, 100, 0, 0)
	FillColor _ (#0000FF)
	Rectangle blue (350, 400, 100, 100, 0, 0)
    FillColor _ (#000000)
    Rectangle line1 (380, 400, 1, 100, 0, 0)
    Rectangle line2 (420, 400, 1, 100, 0, 0)
   
    // Bind a C++ native action
	NativeAction cpp_na (cpp_action, root, 1)
	red.press -> cpp_na

    // Bind a smala native action
	NativeAction smala_na (smala_action, fc, 1)
	green.press -> smala_na

	// Define and bind a smala_lambda (in code native action)
	blue.press -> (root) {
  		// To create a NEW component (either visual or assignment/binding/connector):
        // add it to a parent (passed as the argument of the native, here fc), thanks to
        //     addChildrenTo parent {
        //         ...the new components...
        //     }

        // To set a value on an EXISTING property:
        // do not use an assignment as it would create a new Assignment component
        // and add it to the component tree each time this native is invoked.
        // The value change is propagated.
        root.fc.value = (#0000FF)

        if (root.blue.press.x < 380) {
            root.blue.y = 350
            root.line1.y = 350
            root.line2.y = 350
        } else if (root.blue.press.x > 420) {
            root.blue.y = 450
            root.line1.y = 450
            root.line2.y = 450
        } else {
            root.blue.y = 400
            root.line1.y = 400
            root.line2.y = 400
        }

        // to print the component tree:
        // for (int i = 0; i < 5; i++) {
        //     dump root.fc
        // }
	}
}

