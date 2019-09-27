/*
*  Simple sketching app
*
*  The copyright holders for the contents of this file are:
*  Ecole Nationale de l'Aviation Civile, France (2017)
*  See file "license.terms" for the rights and conditions
*  defined by copyright holders.
*
*
*  Contributors:
*    Jérémie Garcia    <jeremie.garcia@enac.fr>
*    Nicolas Saporito  <nicolas.saporito@enac.fr>
*    Mathieu Magnaudet <mathieu.magnaudet@enac.fr>
*
*/

use core
use base
use display
use gui

import APoint
import DoubleClick


_main_
Component root {
    Frame f ("Sketching", 100, 100, 800, 600)
    Exit ex (0, 1)
    f.close -> ex
    mouseTracking = 1

    // background
    FillColor bgFill (50,50,50)
    Rectangle bgRect (0, 0, 0, 0 , 0, 0)
    f.width =:> bgRect.width
    f.height =:> bgRect.height

    // Foreground styles for sketches
    NoFill nf
    OutlineColor outlineColor (255, 255, 255)
    OutlineWidth outlineWidth (2)

    // content (sketches are drawn here)
    Component content {
        Path lineSegment {
            PathMove origin (0, 0)
            PathLine end (0, 0)
        }
    }

    // clear background on double click
    Spike clear
    DoubleClick doubleClick (bgRect, f)
    doubleClick.double_click -> clear
    clear -> (root) {
        delete root.content

        addChildrenTo root {
            Component content {
                Path lineSegment {
                    PathMove origin (0, 0)
                    PathLine end (0, 0)
                }
            }
        }
    }

    // temp points to draw sketches
    APoint prevPoint (0,0)
    APoint currentPoint (0,0)
    APoint midPoint (0,0)
    APoint prevMidPoint (0,0)

    // set the origin on press
    AssignmentSequence on_press_action (1) {
        f.press.x =: prevPoint.x
        f.press.y =: prevPoint.y
        f.press.x =: prevMidPoint.x
        f.press.y =: prevMidPoint.y
    }

    // Sketch
    FSM sketchFSM {
        State idle
        State sketching {
            f.move.x =:> currentPoint.x
            f.move.y =:> currentPoint.y

            f.move -> (root) {
                //compute mid point              
                root.midPoint.x =  (root.currentPoint.x + root.prevPoint.x) / 2
                root.midPoint.y =  (root.currentPoint.y + root.prevPoint.y) / 2

                addChildrenTo root.content {
                    //quadratic segment between origin and mid point
                    Path quadraticSegment {
                        PathMove origin ($root.prevMidPoint.x, $root.prevMidPoint.y)
                        PathQuadratic end ($root.prevPoint.x, $root.prevPoint.y, $root.midPoint.x, $root.midPoint.y)
                    }
                }

                //assign the values to the line segment
                root.content.lineSegment.origin.x = root.midPoint.x
                root.content.lineSegment.origin.y = root.midPoint.y
                root.content.lineSegment.end.x = root.currentPoint.x
                root.content.lineSegment.end.y = root.currentPoint.y

                //set the previous points values
                root.prevPoint.x = root.currentPoint.x
                root.prevPoint.y = root.currentPoint.y
                root.prevMidPoint.x = root.midPoint.x
                root.prevMidPoint.y = root.midPoint.y
            }
        }

        idle -> sketching (f.press, on_press_action)
        sketching -> idle (f.release)
    }

    // prompt if no strokes
    FSM infoFSM {
        State prompt {
            FillColor w (255, 255, 255)
            TextAnchor anchor (1)
            Text promptText (100, 100, "click and drag to sketch")
            f.width /2 =:> promptText.x
            f.height / 2 =:> promptText.y
        }
        State noprompt 

        prompt -> noprompt (f.press)
        noprompt -> prompt (clear)
    }
}

run root
run syshook
