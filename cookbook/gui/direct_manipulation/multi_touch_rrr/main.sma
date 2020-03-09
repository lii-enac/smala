use core
use base
use display
use gui

_main_
Component root
{
	Frame f ("my frame", 0, 0, 1000, 1000)

	FillColor fc(255,0,0)
  OutlineColor _(0,0,255)

  Component tt {
    Homography h
    for (int i = 0; i < 15; i++) {
      Line _ (400, i * 10 + 400, 550, i * 10 + 400)
      Line _ (i*10 + 400, 400, i*10 + 400, 550)
    }
    NoFill _
    Rectangle r (400, 400, 150, 150, 0, 0)
  }

  Component tt2 {
    Homography h
    for (int i = 0; i < 15; i++) {
      Line _ (0, i * 10, 150, i * 10)
      Line _ (i*10, 0, i*10, 150)
    }
    NoFill _
    Rectangle r (0, 0, 150, 150, 0, 0)
  }
  ScaleRotateTranslate _ (tt.r, tt.h)
  ScaleRotateTranslate _ (tt2.r, tt2.h)
}

run root
run syshook
