use core
use audio
use display
use gui

_main_
 Component root
 {
 	// sadly emscripten audio does not work without gui
 	Frame f("simple audio", 0, 0, 100, 100)
 	Sample s("res/shutter.wav")
 }

run root
run syshook