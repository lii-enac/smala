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

import BoolToggleButton
import FSMGuard

/*
 * FSM transitions and guards:
 * In general a state change is simply triggered by a classical event (for exemple a mouse press on a shape).
 * Sometimes the state change is required on an event, but only if there is also a condition to satisfy. 
 * If the condition is related to the event, then there is generally no need of the notion of FSM guard,
 * because a simple Bool property can serve as a trigger combining event and condition, thanks to its spikes true and false
 * (for example a mouse move superior to 10px to trigger a drag with hysteresis).
 * But if the condition is not related to the event, then it can be interesting to use an intermediate component acting as a guard 
 * (like the FSMGuard component proposed hereafter).
 */
_main_
Component root {
  ///////////
  // frame //
  ///////////

  Frame frame ("my frame", 0, 0, 760, 250)
  Exit ex (0, 1)
  frame.close -> ex

  ///////////////////
  // graphic scene //
  ///////////////////
  
  BoolToggleButton buttonCondition_s1 (50, 50, 125, 50, "Toggle condition s1")
  BoolToggleButton buttonCondition_s2 (50, 150, 125, 50, "Toggle condition s2")

  OutlineColor _ (70, 70, 70)
  FillColor _ (200, 200, 200)
  Rectangle buttonTrigger_s2 (225, 150, 125, 50, 5, 5)
  Rectangle buttonInit (570, 100, 125, 50, 5, 5)

  FillColor _ (70, 70, 70)
  TextAnchor _ (1)
  Text _ (287, 180, "Emit event s2")
  Text _ (440, 130, "FSM state:")
  Text textFsmState (492, 130, "")
  Text _ (632, 130, "Init FSM")

  ///////////
  // logic //
  ///////////

  // Guard component (event + condition)
  FSMGuard guard_s2 (buttonTrigger_s2.press, buttonCondition_s2.value)

  // FSM
  FSM fsmToTest {
    State s0 {
      "s0" =: textFsmState.text
    }

    State s1 {
      // this state will be triggered by a simple boolean "guard"
      // (if BoolToggleButton buttonCondition_s1 value is true)
      "s1" =: textFsmState.text
    }

    State s2 {
      // this state will be triggered by a guard component
      // (on a specific event, but only when BoolToggleButton buttonCondition_s2 value is true)
      "s2" =: textFsmState.text
    }

    // transitions
    s0 -> s1 (buttonCondition_s1.value.true)  // simple boolean "guard" (use the spike true/false from Bool)
    s1 -> s0 (buttonCondition_s1.value.false) // simple boolean "guard" (use the spike true/false from Bool)
    s1 -> s0 (buttonInit.press)               // simple event
    s0 -> s2 (guard_s2.trigger)               // guard component (use the spike trigger from FSMGuard)
    s1 -> s2 (guard_s2.trigger)               // guard component (use the spike trigger from FSMGuard)
    s2 -> s1 (buttonCondition_s1.value.true)  // simple boolean "guard" (use the spike true/false from Bool)
    s2 -> s0 (buttonInit.press)               // simple event
  }

}

