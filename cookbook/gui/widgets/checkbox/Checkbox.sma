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
Checkbox (Component frame, int nb_entry) {
  Int entry (0)
  Spike unselectAll

  entries = repeat (n = nb_entry) {
	Int index (n)
	NoFill nf
	OutlineColor oc (50, 50, 50)
    Circle c (20, n * 15 + 10, 6)
	FillColor black (0, 0, 0)
	Text _label (40, n * 15 + 14, "")
	label aka _label.text
	FSM fsm {
		State st_unselected
		State st_pressed
		State st_selected {
			NoOutline no
			FillColor fc (105, 177, 241)
			Circle s (20, n * 15 + 10, 5)
			index =: entry
		}
		st_unselected->st_pressed (c.press)
		st_pressed->st_selected (c.release, unselectAll)
		st_pressed->st_unselected (frame.release)
		st_selected->st_unselected (unselectAll)
	}
	selected aka fsm.st_selected
  }
}
