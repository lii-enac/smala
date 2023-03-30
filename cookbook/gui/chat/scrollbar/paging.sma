/*
*  djnn Smala compiler
*
*  The copyright holders for the contents of this file are:
*    Ecole Nationale de l'Aviation Civile, France (2017)
*  See file "license.terms" for the rights and conditions
*  defined by copyright holders.
*
*
*  Contributors:
*    Stephane Conversy <stephane.conversy@enac.fr>
*
*/

use core
use base


// paging up and down repeatidly while the user holds a press

_define_
paging(Process model) {
    Double dv (0) // amount of paging in model unit (usually between 0 and 1), must be flowed-in by the client

    Double dvclamped (0)
    Double maxdv(0)
    Double mindv(0)

     (1-model.high) =:> maxdv
          model.low =:> mindv
          
    (dv > 0)
        ?  dv < maxdv ? dv :  maxdv // FIXME: cycle eventually
        : -dv < mindv ? dv : -mindv
                    =:> dvclamped

    // when dv changes, perform an incr operation on the model
    Incr incr_low  (0)
    Incr incr_high (0)

          model.low =: incr_low.state
     incr_low.state =:> model.low

         model.high =: incr_high.state
    incr_high.state =:> model.high
    
          dvclamped =:> incr_low.delta, incr_high.delta
    
    // ... and repeat it as long as the component is activated
    FSM fsm {
        State first {
            Clock clock (250)
            clock.tick -> incr_low, incr_high
        }
        State next_ones {
            Clock clock (80)
            clock.tick -> incr_low, incr_high
        }
        first -> next_ones (first.clock.tick)
   }
}
