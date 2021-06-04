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
	Frame f ("Timed automaton", 0, 0, 370, 300)
	Exit ex (0, 1)
	f.close->ex

	/** UI for event generation */
	FillColor _ (Black)
	Text cl_t (50, 200, "0")
	Text fsm_t (170, 200, "0")
	FillColor _ (100, 100, 100)
	Button ra (f, "Event a", 50, 30)
	Button rb (f, "Event b", 130, 30)
	Button rc (f, "Reset Clock", 210, 30)
	
	/** Graphics **/
	gui = loadFromXML("graphics.svg")
	Translation pos (100, 130)
	Scaling sc (2, 2, 0, 0)
	fsm_ui << gui.layer1.fsm

	Switch sw_colors (s0) {
		Component s0 {
			#CCFFAA =: fsm_ui.s0.fill.value
			#FFFFFF =: fsm_ui.s1.fill.value
		}
		Component s1 {
			#CCFFAA =: fsm_ui.s1.fill.value
			#FFFFFF =: fsm_ui.s0.fill.value
		}
	}
	AssignmentSequence on_t1 (0) {
	  #0000FF =: fsm_ui.up_arrow.up_line.stroke.value, fsm_ui.up_arrow.up_head.stroke.value, fsm_ui.up_arrow.up_head.fill.value
	}
	AssignmentSequence off_t1 (1) {
	  #FF0000 =: fsm_ui.up_arrow.up_line.stroke.value, fsm_ui.up_arrow.up_head.stroke.value, fsm_ui.up_arrow.up_head.fill.value
	}
	AssignmentSequence on_t2 (1) {
		#0000FF =: fsm_ui.down_arrow.down_line.stroke.value, fsm_ui.down_arrow.down_head.stroke.value, fsm_ui.down_arrow.down_head.fill.value
	}
	AssignmentSequence off_t2 (0) {
		#FF0000 =: fsm_ui.down_arrow.down_line.stroke.value, fsm_ui.down_arrow.down_head.stroke.value, fsm_ui.down_arrow.down_head.fill.value
	}

	/** Clock **/
	Clock c (500)
	Incr inc (1)
	inc.delta = 0.1
	x aka inc.state
	c.tick->inc

	AssignmentSequence reset_x (1) {
		0 =: x
	}
	rc.click->reset_x
	
	Bool cond (1)

	/** Automaton **/
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

	/** change transition color on (de)activation **/
	fsm.t2->on_t2
	fsm.t2!->off_t2
	fsm.t1->on_t1
	fsm.t1!->off_t1

	/** display clock x value and automaton state **/
	DoubleFormatter df (0, 1)
	x =:> df.input
	"Clock x = " + df.output =:> cl_t.text
	"Automaton state = " + fsm.state =:> fsm_t.text
	fsm.state =:> sw_colors.state
}