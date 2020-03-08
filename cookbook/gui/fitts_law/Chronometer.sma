use core
use exec_env
use base

_define_
Chronometer (int period)
{
    Counter counter(0, period)

    Switch status(idle) {
      //Component idle {}
      Spike idle
      Component running {
        Clock cl(period)
        cl.tick -> counter.step
      }
    }
    status.idle -> counter.reset

    state aka status.state
    elapsed aka counter.output
}
