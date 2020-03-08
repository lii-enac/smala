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
clamp(Process in, Process min, Process max, Process out) {
	(in < min) ? min : ( in > max ? max : in) =:> out
}
