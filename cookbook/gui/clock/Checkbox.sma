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
 *		Mathieu Magnaudet <mathieu.magnaudet@enac.fr>
 *
 */
 use core
 use base
 use gui

 _define_
 Checkbox (Process frame, int nb_entry, int _x, int _y) {
 	Int entry (0)
 	String choice ("")
 	Spike unselectAll
 	

 	List entries {
 		for (int n = 0; n < nb_entry; n++) {
 			Component _{
 				Spike init
 				Int index (n)
 				NoFill nf
 				OutlineColor oc (127, 127, 127)
 				OutlineWidth ow (3)
 				Circle c (_x + (n * 80), _y, 20)
 				FillColor darkgrey (40, 40, 40)
 				FontSize fs (4, 18)
 				FontFamily ff ("Roboto")
 				TextAnchor ta (1)
 				Text _label (_x + (n * 80), _y + 45, "")
 				label aka _label.text
 				FSM fsm {
 					State st_unselected
 					State st_pressed
 					State st_selected {
 						NoOutline no
 						FillColor fc (105, 177, 241)
 						Circle s (_x + (n * 80), _y, 16)
 						index =: entry
 						label =: choice
 					}
 					st_unselected->st_pressed (c.press)
					st_pressed->st_selected (c.release, unselectAll)
					//st_pressed->st_unselected (c.leave)
					//st_unselected->st_pressed (c.enter)
					st_selected->st_unselected (unselectAll)
					/*
 					st_unselected->st_pressed (c.press)
 					st_unselected->st_selected (init, selected)
 					st_pressed->st_selected (c.release, selected)
 					st_pressed->st_unselected (frame.release)
 					st_selected->st_unselected (unselectAll, unselected)*/
 				}
 				selected aka fsm.st_selected
 			}
 		}
 	}
 }
