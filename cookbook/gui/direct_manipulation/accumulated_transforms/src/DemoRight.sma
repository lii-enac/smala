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
use exec_env
use base
use display
use gui

/*
 * Transformations by accumulation on an Homography. 
 * Right product version:
 * uses the local coordinates system of the node placed just after the Homography (rect)
 */
_define_
DemoRight (Process frame) {

    Component RightTranslateByDemo {
        Translation _ (100, 100)
        Homography transforms                 // transforms are accumulated here
        Rectangle rect (0, 0, 100, 100, 0, 0) // shape to transform (and reference coordinates system)
        Text _ (0, -5, "Translate")
        FSM fsm {
            State idle
            State dragging {
                // Common API for mouse and single touch
                // to react only to mouse, use rect.mouse.[press|enter|move|leave|release]
                rect.move.local_x - rect.press.local_x =:> transforms.rightTranslateBy.dx
                rect.move.local_y - rect.press.local_y =:> transforms.rightTranslateBy.dy
            }
            idle -> dragging (rect.press)
            dragging -> idle (rect.release)
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
                rect.move.local_x ::> transforms.rightRotateBy.cx
                rect.move.local_y ::> transforms.rightRotateBy.cy
                frame.wheel.dy =:> transforms.rightRotateBy.da
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
                rect.move.local_x ::> transforms.rightScaleBy.cx
                rect.move.local_y ::> transforms.rightScaleBy.cy
                Pow p (1.01, 0)
                frame.wheel.dy =:> p.exponent
                p.result =:> transforms.rightScaleBy.sx, transforms.rightScaleBy.sy
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
                rect.move.local_x ::> transforms.rightSkewXBy.cx
                rect.move.local_y ::> transforms.rightSkewXBy.cy
                frame.wheel.dy =:> transforms.rightSkewXBy.da
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
                rect.move.local_x ::> transforms.rightSkewYBy.cx
                rect.move.local_y ::> transforms.rightSkewYBy.cy
                frame.wheel.dy =:> transforms.rightSkewYBy.da
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
            State dragging {
                // Common API for mouse and single touch
                // to react only to mouse, use rect.mouse.[press|enter|move|leave|release]
                rect.move.local_x - rect.press.local_x =:> transforms.rightTranslateBy.dx
                rect.move.local_y - rect.press.local_y =:> transforms.rightTranslateBy.dy
            }
            idle -> dragging (rect.press)
            dragging -> idle (rect.release)
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
                        rect.move.local_x ::> transforms.rightRotateBy.cx
                        rect.move.local_y ::> transforms.rightRotateBy.cy
                        frame.wheel.dy =:> transforms.rightRotateBy.da
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
                        rect.move.local_x ::> transforms.rightScaleBy.cx
                        rect.move.local_y ::> transforms.rightScaleBy.cy
                        Pow p (1.01, 0)
                        frame.wheel.dy =:> p.exponent
                        p.result =:> transforms.rightScaleBy.sx, transforms.rightScaleBy.sy
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