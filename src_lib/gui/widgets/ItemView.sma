use core
use base

_define_
ItemView () {
    RefProperty model (0)
    DerefString model_id (model, "id", DJNN_GET_ON_CHANGE)
    Translation pos (0, 0)
    Int x (0)
    Int y (0)
    Int width (0)
    Int height (0)
    x =:> pos.tx
    y =:> pos.ty
}