/*
*  Smala cookbook fitts_law
*
*  The copyright holders for the contents of this file are:
*    Ecole Nationale de l'Aviation Civile, France (2019-2020)
*  See file "license.terms" for the rights and conditions
*  defined by copyright holders.
*
*
*  Contributors:
*     Magnaudet Mathieu <mathieu.magnaudet@enac.fr>
*
*/
use core
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
