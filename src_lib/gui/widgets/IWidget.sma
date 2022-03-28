use core

_native_code_
%{
#include <iostream>

void
check (Process* p, const string& p_name)
{
	if (p == 0) {
		std::cout << "Process " << p_name << " not found in IWidget initialisation\n";
		exit (0);
	}
}
%}

_define_
IWidget (Process container)
{
	check (container.width, "width")
	check (container.height, "height")
	Double req_width (0)
	Double req_height (0)
	Double min_width (0)
	Double min_height (0)
	Double max_width (0)
	Double max_height (0)

	ClampMin clamp_min_width (0, 0)
	min_width =:> clamp_min_width.min
	ClampMax clamp_max_width (0, 0)
	max_width == 0 ? container.width : max_width =:> clamp_max_width.max

	req_width =:> clamp_min_width.input
	clamp_min_width.result =:> clamp_max_width.input
	width aka clamp_max_width.result

	ClampMin clamp_min_height (0, 0)
	min_height =:> clamp_min_height.min
	ClampMax clamp_max_height (0, 0)
	max_height == 0 ? container.height : max_height =:> clamp_max_height.max

	req_height =:> clamp_min_height.input
	clamp_min_height.result =:> clamp_max_height.input
	height aka clamp_max_height.result

	FontFamily _ ("B612")
    FontSize _ (5, 13) // 5 to use the pixel unit. Allows to have the same rendering on MAC & Linux
    FontWeight _ (25)
}
