use core

_define_
IWidget ()
{
  // specified geometry
 	Int preferred_width (-1)
  Int preferred_height (-1)
  Int min_width (0)
  Int min_height (0)
  Int max_width (16777215)
  Int max_height (16777215)

  Int h_alignment (1) // 0:left 1:center 2:right
  Int v_alignment (1) // 0:top  1:center 2:bottom

  // resulting geometry
  // Double x(0)
  // Double y(0)
  Translation pos (0, 0) // FIXME?: x,y should be plain Double, and there should be no Translation, as some widgets (e.g. scrollbar) need more complex graphical transformations
	x aka pos.tx
	y aka pos.ty
	Double width (0)
	Double height (0)
	
  // actions: activating the following Spikes should trigger something
  Spike enable
  Spike disable

  // signal: something has been triggered, binding to those Spikes should let you know about it
  Spike enabled
  Spike disabled

  // style
  FontFamily font_family ("B612")
  FontSize font_size (5, 13) // 5 to use the pixel unit. Allows to have the same rendering on macOS & Linux
  FontWeight font_weight (25)
}
