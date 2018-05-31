use core
use base
use gui

_native_code_
%{
  extern char* buildPath (const char *file);
%}

  _define_
  Slider (Component frame, double _x, double _y) {
    Translation t (_x, _y)

    x aka t.tx
    y aka t.ty

    string path = CCall (buildPath, "img/slider.svg")
    gslider = loadFromXML (path)

    Spike nop
	Component gobj {
      bkg << gslider.layer1.bkg
      fgd << gslider.layer1.fgd
      Translation t_thumb (94, 0)
      thumb << gslider.layer1.thumb
      t_thumb.tx + thumb.r => fgd.width
      thumb.press->nop
    }
    width aka gobj.bkg.width

    Double output (1)

    Double dx (94)
    Double min (0)
    Double max (94)
    Double range (94)
    Double buff (0)

    gobj.t_thumb.tx / range => buff
    buff < 0 ? 0 : (buff > 1 ? 1 : buff) => output

    FSM exec_fsm {
      State idle
      State dragging {
        Double offset (0)
        gobj.thumb.press.x - gobj.t_thumb.tx =: offset
		dx > min ? (dx < max ? dx : max) : min => gobj.t_thumb.tx
		frame.move.x - offset => dx
        left << gslider.layer1.left
        right << gslider.layer1.right
        Translation value_pos (94, 0)
        value << gslider.layer1.value
        gobj.t_thumb.tx => value_pos.tx
        DoubleFormatter d_to_s (1, 2)
        output => d_to_s.input
        d_to_s.output => value.text
        
      }
      idle->dragging (gobj.thumb.press)
      dragging->idle (frame.release)
    }
    Bool is_right (1)
    Bool is_left (0)
    output == 1 => is_right
    output == 0 => is_left
    FSM pos_state {
      State st_right {
        242 =: exec_fsm.dragging.right.fill.r, exec_fsm.dragging.right.fill.g, exec_fsm.dragging.right.fill.b
      }
      State st_left {
        242 =: exec_fsm.dragging.left.fill.r, exec_fsm.dragging.left.fill.g, exec_fsm.dragging.left.fill.b
      }
      State st_between {
        108 =: exec_fsm.dragging.right.fill.r, exec_fsm.dragging.right.fill.g, exec_fsm.dragging.right.fill.b, exec_fsm.dragging.left.fill.r, exec_fsm.dragging.left.fill.g, exec_fsm.dragging.left.fill.b
      }
      st_left->st_between (is_left.false)
      st_between->st_right (is_right.true)
      st_right->st_between (is_right.false)
      st_between->st_left (is_left.true)
    }
  }