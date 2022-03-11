use core
use base
use audio
use display
use gui

_main_
Component root
{
	// sadly emscripten audio does not work without gui
	Frame f("simple audio", 0, 0, 300, 300)
  	Exit ex (0, 1)
  	f.close -> ex
	
	//Volume v(0.9) // does not work anymore FUTUREWORK

	//0.2 =: DefaultAudioListener.gain

	Sample s("res/shutter.wav")
	//0.2 =: s.gain
	1 =: s.x // only works with mono sounds, does not work with stereo sounds
	//1 =: s.z
	//1.5 =: s.pitch_mul
	//0.5 =: s.lowpass_freq // only works with openal efx support

	Clock cl(2000)
	cl.tick -> s
}
