use core
use base
use animation

_define_
Animator (int duration, double _min, double _max, int func, int loop)
{
  Spike start
  Spike abort
  Spike end
  Spike pause
  Spike resume
  Spike reset
  Double min (_min)
  Double max (_max)
  Double output (0)
  Double min (_min)
  Double max (_max)
  EasingGenerator gen (func)
  Incr inc (1)
  
  inc.state =:> gen.input
  if (loop) {
    inc.state >= 1 -> reset
  } else {
    inc.state >= 1 -> end
  }
  gen.output * (max - min) + min =:> output
 
  FSM fsm {
    State stopped {
      0 =: inc.state 
      0 =: gen.input
    }
    State started {
      Clock cl (60)
      Int num_step (0)
      duration/cl.period =:> num_step
      1 / num_step =:> inc.delta
      cl.tick->inc      
    }
    State paused
    started->stopped (end)
    {started, paused}->stopped (abort)
    stopped->started (start, reset)
    started->paused (pause)
    paused->started (resume)
  }
  fps aka fsm.started.cl.period
  reset -> { 0 =: inc.state 
             0 =: gen.input }
}