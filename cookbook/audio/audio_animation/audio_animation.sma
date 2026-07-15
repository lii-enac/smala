/*
*  djnn Smala compiler
*
*  The copyright holders for the contents of this file are:
*    Ecole Nationale de l'Aviation Civile, France (2020-2022)
*  See file "license.terms" for the rights and conditions
*  defined by copyright holders.
*
*
*  Contributors:
*     Mathieu Magnaudet <mathieu.magnaudet@enac.fr>
*     Mathieu Poirier <mathieu.poirier@enac.fr>
*
*/
use core
use base
use audio
use display
use gui
use animation

import gui.animation.Animator

_main_
Component root
{
	// sadly emscripten audio does not work without gui
	Frame f("audio animation", 0, 0, 300, 300)
  	Exit ex (0, 1)
  	f.close -> ex
	
	//Sample s("../simple_audio/res/al2.wav")
	Sample s("../simple_audio/res/shutter.wav")
	0.2 =: s.gain
	//1 =: s.x
	//1 =: s.z
	//1.5 =: s.pitch_mul
	1 =: s.lowpass_gain
	0.1 =: s.lowpass_freq
	
	Animator an (30000, 2, 1, DJN_OUT_ELASTIC, 0, 1)
	an.output =:> s.x
	an.fps = 20

	//TextPrinter tp
	//an.output => tp.input
	
	Clock cl(2000)
	cl.tick -> s
	cl.tick -> an.reset
}
