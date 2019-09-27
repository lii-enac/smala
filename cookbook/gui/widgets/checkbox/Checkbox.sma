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
use display
use gui

_define_
Checkbox (int nb_entry) {
  Int entry (0)
  Spike unselectAll

  List entries {
	for (int n = 0; n < nb_entry; n++)
		{
			Component _ {
				Int index (n)
				NoFill nf
				OutlineColor oc (250, 50, 50)
    			Circle c (20, n * 15 + 10, 6)
				FillColor black (0, 0, 0)
				Text _label (40, n * 15 + 14, "")
				label aka _label.text
				FSM fsm {
					State st_unselected
					State st_pressed {
						NoOutline no
						FillColor fc (05, 77, 241)
						Circle s (20, n * 15 + 10, 5)
					}
					State st_selected {
						NoOutline no
						FillColor fc (105, 177, 241)
						Circle s (20, n * 15 + 10, 5)
						index =: entry
					}
					st_unselected->st_pressed (c.press)
					st_pressed->st_selected (c.release, unselectAll)
					st_pressed->st_unselected (c.leave)
					st_unselected->st_pressed (c.enter)
					st_selected->st_unselected (unselectAll)
				}
				selected aka fsm.st_selected

				//debug
				LogPrinter lp ("log " + to_string(n) + " :")
				fsm.state => lp.input
			}
  		}
	}
}