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
use gui


_action_
cpp_action (Component c)
%{
	/* how to get the source */
	//Process *source = c->get_activation_source ();
	
	/* how to get the user_data */
 	Process *data = (Process*) get_native_user_data (c);

 	Process *fc = data->find_component ("fc");

 	((IntProperty*) fc->find_component ("r"))->set_value (250, 1)  ;
 	((IntProperty*) fc->find_component ("g"))->set_value (0, 1)  ;
 	((IntProperty*) fc->find_component ("b"))->set_value (0, 1)  ;

    /* debug to check */
 	fc->dump(0);
%}

_action_
smala_action (Component src, Component data)
{   

   /* note: 
    * if you use assignment (->) or binding (=>)
    * they have to be related to a parent
    * here : data 
    * so you have to add them to data with
    * an "addChildrenTo"
    */
    data.r = 0
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

	NativeAction cpp_na (cpp_action, root, 1)
	red.press -> cpp_na

	NativeAction smala_na (smala_action, fc, 1)
	green.press -> smala_na

	/* smala_lambda - in code native action */
	blue.press -> (fc) {
  		/* note: 
    	* if you use assignment (->) or binding (=>)
    	* they have to be related to a parent
    	* here : data 
    	* so you have to add them to data with
    	* an "addChildrenTo"
    	*/
    	fc.r = 0
    	fc.g = 0
    	fc.b = 255

    	dump fc
	}
}

run root
run syshook