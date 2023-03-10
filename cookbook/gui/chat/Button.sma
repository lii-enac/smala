use core
use base
use gui

_define_
Button (string _label, double _x, double _y) {
  /*----- interface -----*/
  Spike click   // FIXME? should be past simple: 'clicked', or better: 'triggered' ("less" linked to the particular interaction that triggered it)
  Spike release // FIXME? should be past simple: 'released'
  Spike select  // _no_ FIXME: this is one is an action(?)
  Spike enable
  Spike disable
  Spike enabled
  Spike disabled
  // Spike trigger // action: should animate a triggering
  // Spike silent_trigger // action: should trigger with no animation

  /*----- interface -----*/

  Translation pos (_x, _y)
  x aka pos.tx
  y aka pos.ty
  Double width (24)
  Double height (24)

  Int idle_color (#6F9BAD)
  Int hover_color (#49707F)
  Int pressed_color (#0190C7)
  Bool ret_key_pressed (0)
  Bool ret_key_released (0)
  GenericKeyboard.key\-pressed == DJN_Key_Return => ret_key_pressed
  GenericKeyboard.key\-released == DJN_Key_Return => ret_key_released
  FSM fsm_availability {
    State disabled
    State enabled {
        svg = load_from_XML ("img/send_button.svg")
        bg << svg.layer1.bg
        fg << svg.layer1.fg
        FSM button_state {
            State idle {
                idle_color =: bg.fill.value
            }
            State hover {
                hover_color =: bg.fill.value
            }
            State pressed {
                pressed_color =: bg.fill.value
                bg.release->release
            }
            State out {
                idle_color =: bg.fill.value
            }
            hover->pressed (bg.press)
            idle->hover (bg.enter)
            hover->idle (bg.leave)
            pressed->hover (bg.release, click)
            pressed->out (bg.leave)
            out->pressed (bg.enter)
            out->idle (bg.release)
        }
    }
    disabled->enabled (this.enable,  this.enabled)
    enabled->disabled (this.disable, this.disabled)
  }
  initial_state aka fsm_availability.initial
}