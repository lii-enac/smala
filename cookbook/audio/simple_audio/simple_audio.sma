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
 	Volume v(0.5)
 	Sample s("res/shutter.wav")
 	Clock cl(2000)
 	cl.tick -> s
 }

run root
run syshook