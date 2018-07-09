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
*
*/

use core
use base
use gui

import APoint
import DoubleClick

_action_
clearContent (Component src1, Component data1)
{   
    root = getRef(data1.rootRef)
    contentToRemove = getRef(data1.contentRef)
    remove contentToRemove from root

    addChildrenTo root {
        Component content {
            NoFill nf
            OutlineColor outlineColor (255,255,255)
            OutlineWidth outlineWidth (2)
        }

        Ref tmpRef (content)
        tmpRef =: data1.contentRef
    }
}

_action_
createSegment (Component src, Component data)
{   
    current = getRef(data.currentRef)
    prev = getRef(data.prevRef)
    content = getRef(data.contentRef)

    addChildrenTo content {
        Path segment {
            PathMove origin (0, 0)
            PathLine end (100, 100)
        }

        //assign the values to the segment
        prev.x =: segment.origin.x
        prev.y =: segment.origin.y
        current.x =: segment.end.x
        current.y =: segment.end.y

        //use the paused assignements to set the previous point value
        current.x :: prev.x
        current.y :: prev.y
    }
}

_main_
Component root {
    Frame f ("Line Sketching", 100, 100, 800, 600)
    Exit ex (0, 1)
    f.close -> ex

    /*----- Background -----*/
    FillColor bgFill (50,50,50)
    Rectangle bgRect (0, 0, 0, 0 , 0, 0)
    f.width => bgRect.width
    f.height => bgRect.height

    DoubleClick doubleClick (bgRect, f)

    APoint prevPoint (0,0)
    APoint currentPoint(0,0)

    //set the origin on press
    Component onPressAction {
        f.press.x =: prevPoint.x
        f.press.y =: prevPoint.y
    }

    Component content {
        NoFill nf
        OutlineColor outlineColor (255, 255, 255)
        OutlineWidth outlineWidth (2)
    }

    Component clear
    Component clearData{
        Ref rootRef (root)
        Ref contentRef (content)
    }

    NativeAction clearAction (clearContent, clearData, 1)
    clear -> clearAction

    doubleClick.doubleClicked -> clear

    FSM sketchFSM{
        State idle
        State sketching{
            f.move.x => currentPoint.x
            f.move.y => currentPoint.y

            //data component with points and content
            Component actionData {
                Ref prevRef (prevPoint)
                Ref currentRef (currentPoint)            
                Ref contentRef (root.content)
                clearData.contentRef => contentRef
            }

            //call this action on each move event
            NativeAction createSegmentAction (createSegment, actionData, 1)
            f.move -> createSegmentAction
        }
        idle -> sketching (f.press, onPressAction)
        sketching -> idle (f.release)
    }

    // prompt if no strokes
    FSM infoFSM {
        State prompt {
            FillColor w (255, 255, 255)
            TextAnchor anchor (1)
            Text promptText (100, 100, "click and drag to sketch")
            f.width /2 => promptText.x
            f.height / 2 => promptText.y
        }
        State noprompt {

        }
        prompt -> noprompt (f.press)
        noprompt -> prompt (clear)
    }
}
run root
run syshook
