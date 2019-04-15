/*
 *	djnn Smala compiler
 *
 *	The copyright holders for the contents of this file are:
 *		Ecole Nationale de l'Aviation Civile, France (2017)
 *	See file "license.terms" for the rights and conditions
 *	defined by copyright holders.
 *
 *
 *	Contributors:
 *		Nicolas Saporito <nicolas.saporito@enac.fr>
 *
 */
use core
use base
use gui

/*
 * Transformations by accumulation on an Homography. 
 * Right product version:
 * uses the local coordinates system of the node placed just after the Homography (rect)
 */
_define_
DemoRight (Component frame) {

    Component RightTranslateByDemo {
        Translation _ (100, 100)
        Homography transforms                 // transforms are accumulated here
        Rectangle rect (0, 0, 100, 100, 0, 0) // shape to transform (and reference coordinates system)
        Text _ (0, -5, "Translate")
        FSM fsm {
            State idle
            State pressed {
                ScreenToLocal s2l (rect)
                frame.press.x => s2l.inX
                frame.press.y => s2l.inY
            }
            State dragging {
                ScreenToLocal s2l (rect)
                frame.move.x => s2l.inX
                frame.move.y => s2l.inY
                s2l.outX - pressed.s2l.outX => transforms.rightTranslateBy.dx
                s2l.outY - pressed.s2l.outY => transforms.rightTranslateBy.dy
            }
            idle -> pressed (rect.press)
            pressed -> idle (frame.release)
            pressed -> dragging (frame.move)
            dragging -> idle (frame.release)
        }
    }

    Component RightRotateByDemo {
        Translation _ (300, 100)
        Homography transforms
        Rectangle rect (0, 0, 100, 100, 0, 0)
        Text _ (0, -5, "Rotate")
        FSM fsm {
            State off
            State on {
                ScreenToLocal s2l (rect)
                frame.move.x => s2l.inX
                frame.move.y => s2l.inY
                s2l.outX ::> transforms.rightRotateBy.cx
                s2l.outY ::> transforms.rightRotateBy.cy
                frame.wheel.dy => transforms.rightRotateBy.da
            }
            off -> on (rect.enter)
            on -> off (rect.leave)
        }
    }

    Component RightScaleByDemo {
        Translation _ (500, 100)
        Homography transforms
        Rectangle rect (0, 0, 100, 100, 0, 0)
        Text _ (0, -5, "Scale")
        FSM fsm {
            State off
            State on {
                ScreenToLocal s2l (rect)
                frame.move.x => s2l.inX
                frame.move.y => s2l.inY
                s2l.outX ::> transforms.rightScaleBy.cx
                s2l.outY ::> transforms.rightScaleBy.cy
                Pow p (1.01, 0)
                frame.wheel.dy => p.exponent
                p.result => transforms.rightScaleBy.sx, transforms.rightScaleBy.sy
            }
            off -> on (rect.enter)
            on -> off (rect.leave)
        }
    }

    Component RightSkewXByDemo {
        Translation _ (200, 300)
        Homography transforms
        Rectangle rect (0, 0, 100, 100, 0, 0)
        Text _ (0, -5, "SkewX")
        FSM fsm {
            State off
            State on {
                ScreenToLocal s2l (rect)
                frame.move.x => s2l.inX
                frame.move.y => s2l.inY
                s2l.outX ::> transforms.rightSkewXBy.cx
                s2l.outY ::> transforms.rightSkewXBy.cy
                frame.wheel.dy => transforms.rightSkewXBy.da
            }
            off -> on (rect.enter)
            on -> off (rect.leave)
        }
    }

    Component RightSkewYByDemo {
        Translation _ (400, 300)
        Homography transforms
        Rectangle rect (0, 0, 100, 100, 0, 0)
        Text _ (0, -5, "SkewY")
        FSM fsm {
            State off
            State on {
                ScreenToLocal s2l (rect)
                frame.move.x => s2l.inX
                frame.move.y => s2l.inY
                s2l.outX ::> transforms.rightSkewYBy.cx
                s2l.outY ::> transforms.rightSkewYBy.cy
                frame.wheel.dy => transforms.rightSkewYBy.da
            }
            off -> on (rect.enter)
            on -> off (rect.leave)
        }
    }

    /*
     * This demo works well aslong as the scalings are homothetic. 
     * If scalings were not homothetic, ulterior rotations would skew the shape,
     * so another approach is required (see rotate_resize cookbook).
     */
    Component RightTranslateRotateScaleByDemo {
        Translation _ (600, 300)
        Homography transforms
        Rectangle rect (0, 0, 100, 100, 0, 0)
        Text _ (0, -5, "TRS")
        FSM fsmDrag {
            State idle
            State pressed {
                ScreenToLocal s2l (rect)
                frame.press.x => s2l.inX
                frame.press.y => s2l.inY
            }
            State dragging {
                ScreenToLocal s2l (rect)
                frame.move.x => s2l.inX
                frame.move.y => s2l.inY
                s2l.outX - pressed.s2l.outX => transforms.rightTranslateBy.dx
                s2l.outY - pressed.s2l.outY => transforms.rightTranslateBy.dy
            }
            idle -> pressed (rect.press)
            pressed -> idle (frame.release)
            pressed -> dragging (frame.move)
            dragging -> idle (frame.release)
        }
        Rectangle buttonRotateOrResize (23, 105, 54, 15, 0, 0)
        FillColor _ (50, 50, 50)
        Text textRotateOrResize (25, 116, "")
        FSM fsmRotateOrResize {
            State rotate {
                "Rotating" =: textRotateOrResize.text
                FSM fsmRot {
                    State off
                    State on {
                        ScreenToLocal s2l (rect)
                        frame.move.x => s2l.inX
                        frame.move.y => s2l.inY
                        s2l.outX ::> transforms.rightRotateBy.cx
                        s2l.outY ::> transforms.rightRotateBy.cy
                        frame.wheel.dy => transforms.rightRotateBy.da
                    }
                    off -> on (rect.enter)
                    on -> off (rect.leave)
                }
            }
            State resize {
                "Resizing" =: textRotateOrResize.text
                FSM fsm {
                    State off
                    State on {
                        ScreenToLocal s2l (rect)
                        frame.move.x => s2l.inX
                        frame.move.y => s2l.inY
                        s2l.outX ::> transforms.rightScaleBy.cx
                        s2l.outY ::> transforms.rightScaleBy.cy
                        Pow p (1.01, 0)
                        frame.wheel.dy => p.exponent
                        p.result => transforms.rightScaleBy.sx, transforms.rightScaleBy.sy
                    }
                    off -> on (rect.enter)
                    on -> off (rect.leave)
                }
            }
            rotate -> resize (buttonRotateOrResize.press)
            resize -> rotate (buttonRotateOrResize.press)
        }
    }
    
}