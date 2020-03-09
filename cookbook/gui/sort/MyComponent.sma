use core
use base
use gui

_define_
MyComponent (double _x, double _y, double _width, double _height, int r)
{
  Translation pos (_x, _y)
  Int move_y_to (_y)
  FillColor color (r, 50, 50)
  Rectangle shape (0, 0, _width, _height, 5, 5)
  width aka shape.width
  height aka shape.height
  Incr incr (1)
  
  FSM anim {
    State st_idle
    State st_move {
      Bool end_anim (0)
      Incr incr_anim (1)      
      Clock cl (20)
      (move_y_to - pos.ty)/20 =: incr.delta
      0 =: incr_anim.state
      pos.ty =: incr.state

      cl.tick->incr, incr_anim
      incr.state =:> pos.ty
      incr_anim.state >= 20 =:> end_anim
    }
    st_idle->st_move (move_y_to)
    st_move->st_idle (st_move.end_anim.true)
  }
}