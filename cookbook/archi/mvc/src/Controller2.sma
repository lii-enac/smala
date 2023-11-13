use core
use base

_define_
Controller2(Process _model, Process _view)
{
  model aka _model
  view aka _view

  Component control {
    view.t_x.wheel.dy -> { view.t_x.wheel.dy + model.x =: model.x}
    view.t_y.wheel.dy -> { view.t_y.wheel.dy + model.y =: model.y}
    view.t_width.wheel.dy -> { view.t_width.wheel.dy + model.width =: model.width}
    view.t_height.wheel.dy -> { view.t_height.wheel.dy + model.height =: model.height}
    model.{x,y,width,height}=:>view.{x,y,width,height}
  }

  Spike about_to_delete
  about_to_delete->(this) {
    delete this.control
    delete this.view
    delete this
  }
}