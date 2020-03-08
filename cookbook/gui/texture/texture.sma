use core
use exec_env
use base
use display
use gui

_main_
Component root
{
  Frame f ("my frame", 0, 0, 600, 300)
  NoOutline _()
 
  Texture t ("texture.png")
  Rectangle r (10, 10, 150, 80, 5, 5)
  Translation _ (70, -90)
  PathClip border {
    PathMove _(100, 110)
    PathArc _(10, 10, 90, 0, 1, 110, 100)
    PathLine _(190, 100)
    PathLine _(280, 180)
    PathLine _(110, 180)
    PathArc _(10, 10, 90, 0, 1, 100, 170)
    PathLine _(100, 110)

    PathMove _(110, 120)
    PathArc _(10, 10, 90, 0, 1, 120, 110)
    PathLine _(180, 110)
    PathLine _(253, 170)
    PathLine _(120, 170)
    PathArc _(10, 10, 90, 0, 1, 110, 160)
    PathLine _(110, 120)
  }


  Path cr {
    PathMove _(100, 110)
    PathArc _(10, 10, 90, 0, 1, 110, 100)
    PathLine _(190, 100)
    PathLine _(280, 180)
    PathLine _(110, 180)
    PathArc _(10, 10, 90, 0, 1, 100, 170)
    PathLine _(100, 110)
  }
}

run root
run syshook
