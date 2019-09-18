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
use display
use gui

/*
 * Transformations by accumulation on an Homography. 
 * Left product version:
 * uses the local coordinates system of the node placed just before the Homography (ref)
 */
_define_
DemoLeft (Process frame) {

    Component LeftTranslateByDemo {
        Translation _ (100, 100)
        Rectangle ref (0, 0, 0, 0, 0, 0)      // reference coordinates system for...
        Homography transforms                 // ...transforms which are accumulated here
        Rectangle rect (0, 0, 100, 100, 0, 0) // shape to transform
        Text _ (0, -5, "Translate")
        Double x0 (0)
        Double y0 (0)
        FSM fsm {
            State idle
            State pressed {
                ScreenToLocal s2l (ref)
                frame.press.x =:> s2l.inX
                frame.press.y =:> s2l.inY
                s2l.outX =:> x0
                s2l.outY =:> y0
            }
            State dragging {
                ScreenToLocal s2l (ref)
                frame.move.x =:> s2l.inX
                frame.move.y =:> s2l.inY
                AssignmentSequence seqX (1) {
                    s2l.outX - x0 =: transforms.leftTranslateBy.dx
                    s2l.outX :: x0
                }
                s2l.outX -> seqX
                AssignmentSequence seqY (1) {
                    s2l.outY - y0 =: transforms.leftTranslateBy.dy
                    s2l.outY :: y0
                }
                s2l.outY -> seqY
            }
            idle -> pressed (rect.press)
            pressed -> idle (frame.release)
            pressed -> dragging (frame.move)
            dragging -> idle (frame.release)
        }
    }

    Component LeftRotateByDemo {
        Translation _ (300, 100)
        Rectangle ref (0, 0, 0, 0, 0, 0)
        Homography transforms
        Rectangle rect (0, 0, 100, 100, 0, 0)
        Text _ (0, -5, "Rotate")
        FSM fsm {
            State off
            State on {
                ScreenToLocal s2l (ref)
                frame.move.x =:> s2l.inX
                frame.move.y =:> s2l.inY
                s2l.outX ::> transforms.leftRotateBy.cx
                s2l.outY ::> transforms.leftRotateBy.cy
                frame.wheel.dy =:> transforms.leftRotateBy.da
            }
            off -> on (rect.enter)
            on -> off (rect.leave)
        }
    }

    Component LeftScaleByDemo {
        Translation _ (500, 100)
        Rectangle ref (0, 0, 0, 0, 0, 0)
        Homography transforms
        Rectangle rect (0, 0, 100, 100, 0, 0)
        Text _ (0, -5, "Scale")
        FSM fsm {
            State off
            State on {
                ScreenToLocal s2l (ref)
                frame.move.x =:> s2l.inX
                frame.move.y =:> s2l.inY
                s2l.outX ::> transforms.leftScaleBy.cx
                s2l.outY ::> transforms.leftScaleBy.cy
                Pow p (1.01, 0)
                frame.wheel.dy =:> p.exponent
                p.result =:> transforms.leftScaleBy.sx, transforms.leftScaleBy.sy
            }
            off -> on (rect.enter)
            on -> off (rect.leave)
        }
    }

    Component LeftSkewXByDemo {
        Translation _ (200, 300)
        Rectangle ref (0, 0, 0, 0, 0, 0)
        Homography transforms
        Rectangle rect (0, 0, 100, 100, 0, 0)
        Text _ (0, -5, "SkewX")
        FSM fsm {
            State off
            State on {
                ScreenToLocal s2l (ref)
                frame.move.x =:> s2l.inX
                frame.move.y =:> s2l.inY
                s2l.outX ::> transforms.leftSkewXBy.cx
                s2l.outY ::> transforms.leftSkewXBy.cy
                frame.wheel.dy =:> transforms.leftSkewXBy.da
            }
            off -> on (rect.enter)
            on -> off (rect.leave)
        }
    }

    Component LeftSkewYByDemo {
        Translation _ (400, 300)
        Rectangle ref (0, 0, 0, 0, 0, 0)
        Homography transforms
        Rectangle rect (0, 0, 100, 100, 0, 0)
        Text _ (0, -5, "SkewY")
        FSM fsm {
            State off
            State on {
                ScreenToLocal s2l (ref)
                frame.move.x =:> s2l.inX
                frame.move.y =:> s2l.inY
                s2l.outX ::> transforms.leftSkewYBy.cx
                s2l.outY ::> transforms.leftSkewYBy.cy
                frame.wheel.dy =:> transforms.leftSkewYBy.da
            }
            off -> on (rect.enter)
            on -> off (rect.leave)
        }
    }

    /*
     * This demo works well as long as the scalings are homothetic. 
     * If scalings were not homothetic, ulterior rotations would skew the shape,
     * so another approach is required (see rotate_resize cookbook).
     */
    Component LeftTranslateRotateScaleByDemo {
        Translation _ (600, 300)
        Rectangle ref (0, 0, 0, 0, 0, 0)
        Homography transforms
        Rectangle rect (0, 0, 100, 100, 0, 0)
        Text _ (0, -5, "TRS")
        Double x0 (0)
        Double y0 (0)
        FSM fsmDrag {
            State idle
            State pressed {
                ScreenToLocal s2l (ref)
                frame.press.x =:> s2l.inX
                frame.press.y =:> s2l.inY
                s2l.outX =:> x0
                s2l.outY =:> y0
            }
            State dragging {
                ScreenToLocal s2l (ref)
                frame.move.x =:> s2l.inX
                frame.move.y =:> s2l.inY
                AssignmentSequence seqX (1) {
                    s2l.outX - x0 =: transforms.leftTranslateBy.dx
                    s2l.outX :: x0
                }
                s2l.outX -> seqX
                AssignmentSequence seqY (1) {
                    s2l.outY - y0 =: transforms.leftTranslateBy.dy
                    s2l.outY :: y0
                }
                s2l.outY -> seqY
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
                        ScreenToLocal s2l (ref)
                        frame.move.x =:> s2l.inX
                        frame.move.y =:> s2l.inY
                        s2l.outX ::> transforms.leftRotateBy.cx
                        s2l.outY ::> transforms.leftRotateBy.cy
                        frame.wheel.dy =:> transforms.leftRotateBy.da
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
                        ScreenToLocal s2l (ref)
                        frame.move.x =:> s2l.inX
                        frame.move.y =:> s2l.inY
                        s2l.outX ::> transforms.leftScaleBy.cx
                        s2l.outY ::> transforms.leftScaleBy.cy
                        Pow p (1.01, 0)
                        frame.wheel.dy =:> p.exponent
                        p.result =:> transforms.leftScaleBy.sx, transforms.leftScaleBy.sy
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