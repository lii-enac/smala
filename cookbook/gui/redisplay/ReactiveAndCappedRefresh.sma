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
 *		Stéphane Conversy <stephane.conversy@enac.fr>
 *      Mathieu Magnaudet <mathieu.magnaudet@enac.fr>
 *
 */


use core
use base
use display
use gui

_define_
ReactiveAndCappedRefresh(Process f) {

	Int refresh_rate(60) // in Hz
	
	Int period(-1) // in ms
	1000/refresh_rate =:> period

	Timer wait (-1)
    period =:> wait.delay

    //0 =: DrawingRefreshManager.auto_refresh

	FSM refresh_fsm {
	    State idle
	    State limit_refresh_rate {
	      Activator _ (DrawingRefreshManager.draw_sync)
	    }
	    State limit_refresh_rate_redisplay_requested_again

	    idle->limit_refresh_rate (DrawingRefreshManager.damaged, wait)
	    limit_refresh_rate->idle (wait.end)
	    limit_refresh_rate->limit_refresh_rate_redisplay_requested_again (DrawingRefreshManager.damaged)
	    limit_refresh_rate_redisplay_requested_again->limit_refresh_rate (wait.end, wait)
	}
}