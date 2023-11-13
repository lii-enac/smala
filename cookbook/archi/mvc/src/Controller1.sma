use core
use base
_define_
Controller1(Process _model, Process _view)
{
  model aka _model
  view aka _view
  
  Component control {
    model.{x,y,width,height}=:>view.r.{x,y,width,height}
    FSM fsm {
      State idle
      State drag {
        Int off_x(0)
        Int off_y(0)
        view.r.press.x - model.x  =: off_x
        view.r.press.y - model.y =: off_y
        view.r.move.x - off_x =:> model.x
        view.r.move.y - off_y =:> model.y
      }
      idle->drag(view.r.press)
      drag->idle(view.r.release)
    }
  }

  Spike about_to_delete
  about_to_delete->(this) {
    delete this.control
    delete this.view
    delete this
  }
}