use core

_define_
IWidget ()
{
 	Int preferred_width (-1)
  Int preferred_height (-1)
  Int min_width (0)
  Int min_height (0)
  Int max_width (-1)
  Int max_height (-1)
  Int h_alignment (1) //0:left 1:center 2:right
  Int v_alignment (1) //0:top  1:center 2:bottom
	Double width (0)
	Double height (0)
	Translation pos (0, 0)
	x aka pos.tx
	y aka pos.ty

  FontFamily font_family ("B612")
  FontSize font_size (5, 13) // 5 to use the pixel unit. Allows to have the same rendering on MAC & Linux
  FontWeight font_weight (25)
}
