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

 	Process *fc = data->find_component ("fc");

 	((IntProperty*) fc->find_component ("r"))->set_value (250, 1)  ;
 	((IntProperty*) fc->find_component ("g"))->set_value (0, 1)  ;
 	((IntProperty*) fc->find_component ("b"))->set_value (0, 1)  ;

    // To print the component tree:
 	fc->dump(0);
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
    data.r =  0
    data.g = 255
    data.b = 0

    dump data
}

_main_
Component root {

	Frame f ("f", 0, 0, 500, 600)

	FillColor fc (255, 255, 255)

	Rectangle _ (200, 100, 100, 100, 0, 0)
	FillColor _ (255, 0, 0)
	Rectangle red (50, 400, 100, 100, 0, 0)
	FillColor _ (0, 255, 0)
	Rectangle green (200, 400, 100, 100, 0, 0)
	FillColor _ (0, 0, 255)
	Rectangle blue (350, 400, 100, 100, 0, 0)

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
        root.fc.r--
        root.fc.g = 0
        root.fc.b = 255
        if (root.blue.press.x < 400) {
            root.blue.y = 100
        } else if (root.blue.press.x > 430) {
            root.blue.y = 500
        } else {
            root.blue.y = 400
        }

        for (int i = 0; i < 5; i++) {
            dump root.fc
        }
	}
}

run root
run syshook
