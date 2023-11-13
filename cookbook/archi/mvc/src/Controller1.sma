use core
use base
use gui

_define_
Controller1(Process _model, Process _view)
{
  model aka _model
  view aka _view
  
  Spike about_to_delete

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

    FSM color_mngt {
      State unselected {
        #FF0000 =: view.fill.value
      }
      State selected {
        #00FF00 =: view.fill.value
        GenericKeyboard.key\-pressed == DJN_Key_Delete->about_to_delete
      }
      unselected->selected(view.r.press)
      selected->unselected(view.r.press)
    }
  }


  about_to_delete->(this) {
    delete this.control
    delete this.view
    delete this
  }
}