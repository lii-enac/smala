use core
use base
use gui
use animation

import gui.widgets.HBox
import gui.widgets.VBox
import gui.widgets.PushButton
import gui.widgets.Label
import gui.animation.Animator
//import gui.widgets.StandAlonePushButton
import gui.widgets.UITextField
import scrollbar.Scrollbar

import Bubble
import MyTextField
import Button

_native_code_
%{
#include "cmath"
using std::max;

#include "core/utils/to_string.h"
%}

_define_
Chat (double _x, double _y, double _width, double _height, Process frame) {
    Double x(_x)
    Double y(_y)
    Double width(_width)
    Double height(_height)

    Translation pos (_x, _y)
    FillColor _(White)
    Rectangle bg_dial (0, 0, 390, 486)
    Rectangle _ (390, 0, 10, 486)
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
    bg_dial.width + 5 =:> sb.x
    MyTextField edit (0, 520, 400)
    
    bg_dial.height - 13 =:> edit.y
    Button send ("send", 420, 519)
    send.initial_state = "disabled"
    edit.width - 24 =:> send.x
    edit.y + 2 =:> send.y
    bg_dial.width + 10 =:> edit.width
    toString(edit.field.content.text) != "" -> send.enable
    toString(edit.field.content.text) == "" -> send.disable

    // Switch templates("enabled") {
    //     Component enabled
    //     Component disabled {
    //         Bubble bubble ("empty", 0,0,0)
    //     }
    // }
    //msg_height aka templates.disabled.bubble.height
    int msg_height = 20
    int msg_sep = 5

    180 =: sb.transform.rot
    sb.height + 10 =: sb.y
    0 =: sb.pick_xoffset

    // sb model 0: first message 1:last message

    addChildrenTo this.conversation {
        for (int i = 0; i < 25; i++) {
            string msg = "msg" // + to_string(i+1)
            Bubble b ($msg, i*(msg_height+msg_sep), 1, this.bg_dial)
        }
    }

    Double num_visible_msgs(0)
    sb.height / (msg_height + msg_sep) =:> num_visible_msgs

    // initial positionning
    Double initial_low(0)
    max(0., 1 - $num_visible_msgs / $conversation.size) =: initial_low

    initial_low =: sb.model.low
    1 =: sb.model.high

    // permanent positioning relationship
    Double offset (0)
    (num_visible_msgs > $conversation.size
        ? ((msg_height+msg_sep) * ($num_visible_msgs - $conversation.size))
        : 0)
    =:> offset

    offset
    - sb.model.low * (msg_height+msg_sep) * ($conversation.size) - msg_sep =:> conversation_tr.ty

    TextPrinter tp
    //sb.model.low =:> tp.input
    //num_visible_msgs =:> tp.input
    //offset =:> tp.input

    
    send.click -> add_msg: (this) {
        int sz = $this.conversation.size
        int y = 0
        int msg_height = 20
        int msg_sep = 5
        if (sz != 0) {
            y = this.conversation.[sz].y + msg_sep + msg_height // this.conversation.[sz].height + msg_sep
        }

        addChildrenTo this.conversation {
            //int sz = $this.conversation.size
            string txt = toString(this.edit.field.content.text) // + to_string(sz+1)
            Bubble b (txt, y, 0, this.bg_dial)
            //b.ui.text = 
            //b.v_alignment = 2 // FIXME does not seem to be working
        }
    }
    edit.validate -> add_msg
    add_msg -> edit.clear


    Animator anim (100, 0, 1, DJN_IN_OUT_QUAD, 0, 0)
    anim.output => sb.model.low
    add_msg -> start_anim: {
        sb.model.low =: anim.min
        max(0., 1 - $num_visible_msgs / $conversation.size) =: anim.max
    }
    start_anim -> anim.reset, anim.start
    //sb.model.low =:> tp.input

    // clip messages by disabling them
    sb.model.low -> (this) {
        int nb_items = this.conversation.size
        double v = 0
        double acc = 1. 
        acc = acc/nb_items
        for item : this.conversation {
            //printf("%f %f\n", acc, v)
            if ( v>= ($this.sb.model.low-1) && v <= ($this.sb.model.high+1) ) { // +1 and -1: display partially appearing bubble
                //item.bg_color.b = 255
                activate(item)
            } else {
                //item.bg_color.b = 100
                deactivate(item)
            }
            v += acc
        }
    }

}