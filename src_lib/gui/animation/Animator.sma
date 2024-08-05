use core
use base
use animation

_define_
Animator (int _duration, double _min, double _max, int func, int _loop, int _start_on_activation)
{
  Spike start
  Spike abort
  Spike end
  Spike pause
  Spike resume
  Spike reset
  Spike rewind

  Int duration (_duration)
  Double min (_min)
  Double max (_max)
  Double output (0)
  EasingGenerator gen (func)
  Incr incr (1)
  //Int loop (_loop)
  //Int start_on_activation (_start_on_activation)

  PreviousDouble previous (0)

  // TextPrinter tp
  
  incr.state =:> gen.input, previous.input
 
  FSM fsm {
    State stopped {
      // Reset
      0 =: incr.state
      0 =: gen.input

      // Option 1.1: First, bind the check on the flag "start_on_activation"
      // start_on_activation == 1 -> start

      if (_start_on_activation) {
        // Option 1.2: Then, set the flag "start_on_activation"
        // 1 =: start_on_activation

        // Option 2
        |->start
      }
    }

    State started {
      // Option 1: Reset the flag "start_on_activation"
      // 0 =: start_on_activation

      Clock cl (20)
      Int num_step (0)
      duration/cl.period =:> num_step
      1 / num_step =:> incr.delta
      cl.tick -> incr
      gen.output * (max - min) + min =:> output

      if (_loop) {
        incr.state >= 1 -> reset
      }
      else {
        incr.state >= 1 -> end
      }
    }

    State rewinding {
      Clock cl (20)
      Bool is_ended (0)
      // Use previous value, because incr.state may have been reseted in state "stopped"
      previous.output =: incr.state
      -incr.delta =: incr.delta
      cl.tick -> incr
      gen.output * (max - min) + min =:> output

      incr.state <= 0 => is_ended
    }
    
    State paused

    started -> stopped (end)
    {started, paused} -> stopped (abort)
    
    {started, paused, stopped} -> rewinding (rewind)
    rewinding -> stopped (rewinding.is_ended.true, end)
    rewinding -> started (start)
    
    stopped -> started (start, reset)
    started -> paused (pause)
    paused -> started (resume)
  }

  //"start_on_activation " + start_on_activation + " - state " + fsm.state =:> tp.input
  //"State " + fsm.state =:> tp.input
  
  fps aka fsm.started.cl.period

  reset -> {
    0 =: incr.state 
    0 =: gen.input
  }
}