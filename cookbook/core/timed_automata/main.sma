/*
 *	djnn Smala compiler
 *
 *	The copyright holders for the contents of this file are:
 *		Ecole Nationale de l'Aviation Civile, France (2021)
 *	See file "license.terms" for the rights and conditions
 *	defined by copyright holders.
 *
 *
 *	Contributors:
 *		Mathieu Magnaudet <mathieu.magnaudet@enac.fr>
 *
 * 
 * This is an example of how to implement the first 
 * timed automaton found (fig. 3) in the paper:
 * Alur, R. & Dill, D. L., A theory of timed automata, Th. Comp. Sc. 126 (1994)
 *
 */

use core
use base
use gui

import gui.widgets.Button

_main_
Component root {
	Frame f ("Timed automata", 0, 0, 400, 400)
	
	/** UI for event generation */
	FillColor _ (Black)
	Text t (30, 30, "0")
	FillColor _ (100, 100, 100)
	Button ra (f, "Event a", 50, 50)
	Button rb (f, "Event b", 130, 50)
	Button rc (f, "Reset Clock", 210, 50)
	
	/* Clock */
	Clock c (1000)
	Incr inc (1)
	x aka inc.state
	c.tick->inc

	AssignmentSequence reset_x (1) {
		0 =: x
	}
	rc.click->reset_x
	
	Bool cond (1)

	FSM fsm {
		State s0
		State s1 {
			x < 2 =:> cond
		}
		t1:s0->s1 (ra.click, reset_x)
		t2:s1->s0 (rb.click)
	}

	cond.true->fsm.t2
	cond.false->!fsm.t2

	"x = " + x + " state = " + fsm.state =:> t.text
}