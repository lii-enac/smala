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
    Frame f ("Line Sketching", 100, 100, 800, 600)
    Exit ex (0, 1)
    f.close -> ex
    mouseTracking = 1;

    // Background
    FillColor bgFill (50, 50,50)
    Rectangle bgRect (0, 0, 0, 0, 0, 0)
    f.width =:> bgRect.width
    f.height =:> bgRect.height

    // Foreground styles for sketches
    NoFill nf
    OutlineColor outlineColor (255, 255, 255)
    OutlineWidth outlineWidth (2)

    // Content to host sketches
    Component content

    // Clear background on double click
    Spike clear
    DoubleClick doubleClick (bgRect, f)
    doubleClick.double_click -> clear
    clear -> (root) {
        delete root.content

        addChildrenTo root {
            Component content 
        }
    }

    // Sketch
    APoint currentPoint(0, 0)
    APoint prevPoint (0, 0)

    FSM sketchFSM {
        State idle

        State sketching {
            f.press.x =: prevPoint.x
            f.press.y =: prevPoint.y

            f.move.x =:> currentPoint.x
            f.move.y =:> currentPoint.y

            // Call this action on each move event
            f.move -> (root) {
                addChildrenTo root.content {
                    Path segment {
                        PathMove origin (0, 0)
                        PathLine end (100, 100)
                    }

                    // Assign the values to the segment
                    // Get primitive double value from a djnn DoubleProperty,
                    // then set this value on an already existing property 
                    // (do not use an assignment as it would create a new Assignment component
                    //  and add it to the component tree each time this native is invoked)
                    segment.origin.x = root.prevPoint.x
                    segment.origin.y = root.prevPoint.y

                    segment.end.x = root.currentPoint.x
                    segment.end.y = root.currentPoint.y

                    // Set the previous point value to prepare the next call
                    root.prevPoint.x = root.currentPoint.x
                    root.prevPoint.y = root.currentPoint.y
                }
            }
        }

        idle -> sketching (f.press)
        sketching -> idle (f.release)
    }

    // Prompt if no strokes
    FSM infoFSM {
        State prompt {
            FillColor w (255, 255, 255)
            TextAnchor anchor (1)
            Text promptText (100, 100, "click and drag to sketch")
            f.width / 2 =:> promptText.x
            f.height / 2 =:> promptText.y
        }

        State noprompt

        prompt -> noprompt (f.press)
        noprompt -> prompt (clear)
    }
}

run root
run syshook
