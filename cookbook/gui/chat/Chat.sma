use core
use base
use gui

import gui.widgets.HBox
import gui.widgets.VBox
import gui.widgets.PushButton
import gui.widgets.Label
//import gui.widgets.StandAlonePushButton
import gui.widgets.UITextField
import scrollbar.Scrollbar

import Bubble
import MyTextField
import Button

_define_
Chat (double _x, double _y, Process frame) {

    //RectangleClip cpr_(0,0,800,800) // does not really work in a responsive interface...
    Translation pos (_x, _y)
    FillColor _(White)
    Rectangle bg_dial (0, 0, 400, 486)
    Component sub {
        RectangleClip clip (0, 0, 300, 350)
        bg_dial.{x,y,width} =:> clip.{x,y,width}
        bg_dial.height - 26 =:> clip.height
        Translation conversation_tr (0, 0)
        List conversation
    }
    conversation aka sub.conversation
    conversation_tr aka sub.conversation_tr

    Scrollbar sb (400, 0, bg_dial, frame)
    bg_dial.height - 36 =:> sb.height
    bg_dial.width - 7 =:> sb.x
    MyTextField edit (0, 520, 400)
    
    bg_dial.height - 13 =:> edit.y
    Button send ("send", 420, 519)
    send.initial_state = "disabled"
    edit.width - 24 =:> send.x
    edit.y + 2 =:> send.y
    bg_dial.width =:> edit.width
    toString(edit.field.content.text) != "" -> send.enable
    toString(edit.field.content.text) == "" -> send.disable

    /*
    conversation aka this.main_box.items.[1].items.[1]
    conversation_tr aka this.main_box.items.[1].items.[1].g.tr
    sb aka this.main_box.items.[1].items.[2]
    edit  aka this.main_box.items.[2].items.[1]
    send aka this.main_box.items.[2].items.[2]
    new_msg aka this.main_box.items.[2]*/

    /*
    
    300 =: conversation.min_width  //= 300
    100 =: conversation.min_height  //= 100

    0 =: sb.model.low
    1 =: sb.model.high

    10  =: sb.preferred_width
    100 =: sb.preferred_height
    sb.h_alignment = 2
    2 =: new_msg.v_alignment

    Timer t(0)
    t.end -> send.disable

    toString(edit.field.content.text) != "" -> send.enable
    toString(edit.field.content.text) == "" -> send.disable
    */
    addChildrenTo this.conversation {
        Bubble b ("initial commit", 0, 0, 0)
        //b.ui.text = 
        //b.v_alignment = 2 // FIXME does not seem to be working
    }
    send.click -> add_msg: (this) {
        int sz = $this.conversation.size
        int y = 0
        if (sz != 0) {
            y = this.conversation.[sz].y + this.conversation.[sz].height + 5
        }

        addChildrenTo this.conversation {
            string txt = toString(this.edit.field.content.text)
            Bubble b (txt, 0, y, 1)
            //b.ui.text = 
            //b.v_alignment = 2 // FIXME does not seem to be working
        }
    }
    add_msg -> edit.clear
    edit.validate -> add_msg
    ((conversation.size > 2) && sb.model.low == 0) -> {
        2.0 / conversation.size =: sb.model.high
    }
    ((conversation.size > 2) && sb.model.low > 1) -> {
        sb.model.low + 2.0 / conversation.size =: sb.model.high
    }

    //_DEBUG_GRAPH_CYCLE_DETECT = 1

    // clip messages by disabling them
    sb.model.low -> (this) {
        int nb_items = this.conversation.size
        double v = 0
        double acc = 1. 
        acc = acc/nb_items
        for item : this.conversation {
            //printf("%f %f\n", acc, v)
            if ( v>= $this.sb.model.low && v <= $this.sb.model.high) {
                //item.bg_color.b = 255
                activate(item)
            } else {
                //item.bg_color.b = 100
                deactivate(item)
            }
            v += acc
        }
    }

    sb.model.low * 100 =:> conversation_tr.ty

}