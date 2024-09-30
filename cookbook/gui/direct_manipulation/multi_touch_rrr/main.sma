use core
use base
use display
use gui

_main_
Component root
{
  //activate touches
  _DEBUG_NO_TOUCH_EVENT = 0

	Frame f ("Multitouch rrr", 0, 0, 1000, 1000)
  Exit ex (0, 1)
  f.close -> ex
	
  OutlineColor _(0,255,255)

  Component rrr_rectangle1 {
    Homography h
    FillColor _ (255, 0, 0)
    Rectangle r (400, 400, 150, 150, 0, 0)
  }

  Component rrr_rectangle2 {
    Homography h
    FillColor _ (0, 255, 0)
    Rectangle r (0, 0, 150, 150, 0, 0)
  }
  ScaleRotateTranslate _ (rrr_rectangle1.r, rrr_rectangle1.h)
  ScaleRotateTranslate _ (rrr_rectangle2.r, rrr_rectangle2.h)
}