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
use exec_env
use base


_define_
forward_transform(Process transform, Process in, Process x, Process y) { // WARNING: not tested
	( (in * transform.s) + transform.tx ) * transform.cosa =:> x
	( (in * transform.s) + transform.ty ) * transform.sina =:> y
}

_define_
inverse_transform(Process transform, Process x, Process y, Process out) {
	( (- x * transform.sina + y * transform.cosa) - transform.ty) / transform.s =:> out
}
