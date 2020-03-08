use core
use exec_env
use base
use gui

_define_
mspf (Process f)
{
    FillColor fc (255, 255, 255)
    Text txt_mspf (30,30, "mspf: ")
    FillColor fc (255, 0, 0)
	Text mspf (68, 30, "0")
    f.mspf => mspf.text
}