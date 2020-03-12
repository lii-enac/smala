use core
use base
use audio
use display
use gui

_main_
Component root
{
	// sadly emscripten audio does not work without gui
	Frame f("simple audio", 0, 0, 100, 100)
  Exit ex (0, 1)
  f.close -> ex
	Volume v(0.5)
	0.9 =: DefaultAudioListener.gain
	Sample s("res/shutter.wav")
	Clock cl(2000)
	cl.tick -> s
}
