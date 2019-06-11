use core
use base
use gui


_native_code_
%{
  extern char* buildPath (const char *file);
%}


_define_
Slider (Process frame, double _x, double _y) {
  Translation t (_x, _y)
  x aka t.tx
  y aka t.ty

  gslider = loadFromXML (buildPath ("img/slider.svg"))

  Component gobj {
    bkg << gslider.layer1.bkg
    fgd << gslider.layer1.fgd
    Translation t_thumb (94, 0)
    thumb << gslider.layer1.thumb
    t_thumb.tx + thumb.r =:> fgd.width
  }
  width aka gobj.bkg.width

  Double output (1)

  Double dx (94)
  Double min (0)
  Double max (94)
  Double range (94)
  Double buff (0)

  gobj.t_thumb.tx / range =:> buff
  buff < 0 ? 0 : (buff > 1 ? 1 : buff) =:> output

  FSM exec_fsm {
    State idle
    State dragging {
      Double offset (0)
      gobj.thumb.press.x - gobj.t_thumb.tx =: offset
	    dx > min ? (dx < max ? dx : max) : min =:> gobj.t_thumb.tx
	    frame.move.x - offset =:> dx
      left << gslider.layer1.left
      right << gslider.layer1.right
      Translation value_pos (94, 0)
      value << gslider.layer1.value
      gobj.t_thumb.tx =:> value_pos.tx
      DoubleFormatter d_to_s (1, 2)
      output =:> d_to_s.input
      d_to_s.output =:> value.text
    }
    
    idle -> dragging (gobj.thumb.press)
    dragging -> idle (frame.release)
  }

  Switch pos (right) {
    Component right
    Component left
    Component between
  }
  output == 1 ? "right" : (output == 0 ? "left" : "between") =:> pos.state

  FSM pos_state {
    State right {
      242 =: exec_fsm.dragging.right.fill.r, exec_fsm.dragging.right.fill.g, exec_fsm.dragging.right.fill.b
    }
    State left {
      242 =: exec_fsm.dragging.left.fill.r, exec_fsm.dragging.left.fill.g, exec_fsm.dragging.left.fill.b
    }
    State between {
      108 =: exec_fsm.dragging.right.fill.r, exec_fsm.dragging.right.fill.g, exec_fsm.dragging.right.fill.b, exec_fsm.dragging.left.fill.r, exec_fsm.dragging.left.fill.g, exec_fsm.dragging.left.fill.b
    }

    left -> between  (pos.between)
    between -> right (pos.right)
    right -> between (pos.between)
    between -> left  (pos.left)
  }
}