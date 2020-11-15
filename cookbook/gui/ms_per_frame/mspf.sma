use core
use base
use gui

_define_
mspf (Process f)
{
    FillColor _ (255, 255, 255)
    Text txt_mspf (30,30, "mspf: ")
    FillColor _ (255, 0, 0)
	Text mspf (68, 30, "0")
    f.mspf => mspf.text
}