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
    f.width => bgRect.width
    f.height => bgRect.height

    // Content to host sketches
    Component content {
        NoFill nf
        OutlineColor outlineColor (255, 255, 255)
        OutlineWidth outlineWidth (2)
    }
    Ref contentRef (content)

    // Clear background on double click
    Spike clear
    DoubleClick doubleClick (bgRect, f)
    doubleClick.double_click -> clear
    clear -> (root) {
        contentToRemove = getRef(root.contentRef)
        remove contentToRemove from root

        addChildrenTo root {
            Component content {
                NoFill nf
                OutlineColor outlineColor (255, 255, 255)
                OutlineWidth outlineWidth (2)
            }

            Ref tmpRef (content)
            tmpRef =: root.contentRef
        }
    }

    // Sketch
    APoint prevPoint (0, 0)
    Ref prevRef (prevPoint)
    APoint currentPoint(0, 0)
    Ref currentRef (currentPoint)

    FSM sketchFSM {
        State idle

        State sketching {
            f.press.x =: prevPoint.x
            f.press.y =: prevPoint.y

            f.move.x => currentPoint.x
            f.move.y => currentPoint.y

            // Call this action on each move event
            f.move -> (root) {
                prev = getRef(root.prevRef)
                current = getRef(root.currentRef)
                content = getRef(root.contentRef)

                addChildrenTo content {
                    Path segment {
                        PathMove origin (0, 0)
                        PathLine end (100, 100)
                    }

                    // Assign the values to the segment
                    prev.x =: segment.origin.x
                    prev.y =: segment.origin.y
                    current.x =: segment.end.x
                    current.y =: segment.end.y

                    // Use the paused assignements to set the previous point value
                    current.x :: prev.x
                    current.y :: prev.y
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
            f.width / 2 => promptText.x
            f.height / 2 => promptText.y
        }

        State noprompt

        prompt -> noprompt (f.press)
        noprompt -> prompt (clear)
    }
}

run root
run syshook
