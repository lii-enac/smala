use core
use base
use animation

_define_
Animator (int _duration, double _min, double _max, int func, int loop, int start_on_activation)
{
  Spike start
  Spike abort
  Spike end
  Spike pause
  Spike resume
  Spike reset
  Spike rewind
  Double min (_min)
  Double max (_max)
  Double output (0)
  Int duration (_duration)
  EasingGenerator gen (func)
  Incr inc (1)
  
  inc.state =:> gen.input
  if (loop) {
    inc.state >= 1 -> reset
  } else {
    inc.state >= 1 -> end
  }
  
 
  FSM fsm {
    State stopped
    State started {
      Clock cl (20)
      Int num_step (0)
      duration/cl.period =:> num_step
      1 / num_step =:> inc.delta
      cl.tick->inc
      gen.output * (max - min) + min =:> output
    }
    State rewinding {
      Clock cl (20)
      Bool end (0)
      -inc.delta =: inc.delta
      inc.state <= 0 => end
      cl.tick->inc
      gen.output * (max - min) + min =:> output

    }
    State paused
    started->stopped (end)
    {started, paused}->stopped (abort)
    {started, paused, stopped}->rewinding (rewind)
    rewinding->stopped (rewinding.end.true, end)
    rewinding->started (start)
    stopped->started (start, reset)
    started->paused (pause)
    paused->started (resume)
  }
  fps aka fsm.started.cl.period
  reset -> { 0 =: inc.state 
             0 =: gen.input }
  if (start_on_activation) {
    fsm.initial = "started"
  }
}