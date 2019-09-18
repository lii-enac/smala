/*
 *	djnn Smala compiler
 *
 *	The copyright holders for the contents of this file are:
 *		Ecole Nationale de l'Aviation Civile, France (2017)
 *	See file "license.terms" for the rights and conditions
 *	defined by copyright holders.
 *
 *
 *	Contributors:
 *		Nicolas Saporito <nicolas.saporito@enac.fr>
 *
 */

use core
use base
use display
use gui

/*
 * Component acting as a guard for a FSM: triggers a state change on a specific event, only if a condition is verified.
 * This component is useful only if the condition is not related to the event.
 */
_define_
FSMGuard (Process event, Process condition) {

  /* --- Interface --- */
  Spike trigger // The spike to bind to in order to trigger the required conditional state change in the managed FSM
  /* - End interface - */


  FSM guardFsm {
    State off

    State guard {
      // Transitional state to evaluate the condition of transition in the managed FSM:
      // - condition value is copied into transitionalCondition,
      // - the result immediately triggers a state change in this FSM, either go to "on" or go back to "off" (see transitions below)
      // By construction of djnn FSM, no other instructions will be taken into acount in this transitional state.
      Bool transitionalCondition (0)
      condition =: transitionalCondition
    }

    State on {
      // Transitional state to trigger the required conditional state change in the managed FSM.
      // This trigger also immediately provokes a return to state "off" in this FSM (see transitions below).
      Activator _ (trigger)
    }

    // transitions
    off -> guard (event)
    guard -> off (guard.transitionalCondition.false)
    guard -> on (guard.transitionalCondition.true)
    on -> off (trigger)
  }

}