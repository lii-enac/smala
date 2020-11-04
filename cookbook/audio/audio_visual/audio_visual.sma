use core
use base
use audio
use display
use gui
use animation

_main_
Component root
{
	// sadly emscripten audio does not work without gui
	Frame f("simple audio", 0, 0, 400, 400)
  	Exit ex (0, 1)
  	f.close -> ex


	Sample s("res/al-3-mono.wav")
	0.5 =: s.gain
	  1 =: s.x
	  1 =: s.z
	  1 =: s.lowpass_freq
	  1 =: s.lowpass_gain

	FillColor fill (200,100,100)
	Circle c (100,100,20)

	//property to reuse
	Double ms_time (0)
	Spike start_sound_anim

	Int anim_duration (10000) //ms
	Int anim_step (10) //ms
	Incr anim_inc (0)

	AssignmentSequence restart_animation (1) {
		  0 =: anim_inc.state
		0.5 =: s.gain
		 20 =: c.r
	}

	anim_inc.state * anim_step =:> ms_time
	ms_time >= anim_duration -> restart_animation

	Clock cl($anim_step)
	cl.tick -> anim_inc


	//animation parameters (sinusoids)
	Double mod_freq (2)
	Sine mod (0)
	Abs abs (0)

	(($mod_freq*ms_time/1000) * 3.14) =:> mod.input
	mod.output =:> abs.input

	//Mapping
	abs.output * 0.1 =:> s.lowpass_freq
	abs.output*20 + 20 => c.r

	TextPrinter tp
	//abs.output => tp.input
	//ms_time >= anim_duration => tp.input

	//s !-> s
	1 =: s.loop

	Double range_x (0.5)
	Double range_y (0.5)

	f.move.x / f.width =:> range_x
	f.move.y / f.height =:> range_y

	range_x * 2 + 0.5 =:> s.pitch_mul
	range_y * 5 =:> mod_freq
}
