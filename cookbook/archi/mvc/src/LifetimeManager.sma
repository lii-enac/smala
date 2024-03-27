use core
use base

_define_
LifetimeManager(Process _model)
{
  List controllers
  model aka _model

  Spike about_to_delete
  about_to_delete->(this) {
    for item:this.controllers {
      delete item.control
      delete item.view
      delete item
    }
    delete this.model
    delete this
  }
}