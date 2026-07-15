/*
*  djnn Smala compiler
*
*  The copyright holders for the contents of this file are:
*    Ecole Nationale de l'Aviation Civile, France (2020)
*  See file "license.terms" for the rights and conditions
*  defined by copyright holders.
*
*
*  Contributors:
*     Mathieu Poirier <mathieu.poirier@enac.fr>
*     Magnaudet Mathieu <mathieu.magnaudet@enac.fr>
*
*/
use core
use base
use gui

_define_
mspf (Process f)
{
    FillColor _ (255, 255, 255)
    Text txt_mspf (30,30, "mspf: ")
    FillColor _ (255, 0, 0)
	Text mspf (68, 30, "0")
    f.mspf => mspf.text
}