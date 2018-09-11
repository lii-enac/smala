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

_action_
clearContent (Process src1, Process data1)
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
createSegment (Process src, Process data)
{   
    current = getRef(data.currentRef)
    mid = getRef (data.midRef)
    prev = getRef(data.prevRef)
    prevMid = getRef(data.prevMidRef)
    content = getRef(data.contentRef)
    tmpShape = getRef (data.tmpShapeRef)
    remove tmpShape from content

    addChildrenTo content {
        //compute mid point
        (current.x + prev.x) / 2 =: mid.x
        (current.y + prev.y) / 2 =: mid.y

        Component quadraticSegment {
            //quadratic segment between origin and mid point
            Path quad {
                PathSubpath origin (0, 0)
                PathQuadratic end (0, 0, 0, 0)
            }

            //assign the values to the segment
            prevMid.x =: quad.origin.x
            prevMid.y =: quad.origin.y
            mid.x =: quad.end.x
            mid.y =: quad.end.y
            prev.x =: quad.end.x1
            prev.y =: quad.end.y1
        }

        Component lineSegment {
            //line segment between origin and mid point
            Path line {
                PathSubpath origin (0, 0)
                PathLine end (0, 0)
            }

            //assign the values to the segment
            mid.x =: line.origin.x
            mid.y =: line.origin.y
            current.x =: line.end.x
            current.y =: line.end.y
        }

        Ref tmpRef (lineSegment)
        tmpRef =: data.tmpShapeRef

        //use the paused assignements to set the previous points values
        current.x :: prev.x
        current.y :: prev.y

        mid.x :: prevMid.x
        mid.y :: prevMid.y
    }
}

_main_
Component root {
    Frame f ("Sketching", 100, 100, 800, 600)
    Exit ex (0, 1)
    f.close -> ex
    mouseTracking = 1

    // background
    FillColor bgFill (50,50,50)
    Rectangle bgRect (0, 0, 0, 0 , 0, 0)
    f.width => bgRect.width
    f.height => bgRect.height

    // content (sketches are drawn here)
    Component content {
        NoFill nf
        OutlineColor outlineColor (255,255,255)
        OutlineWidth outlineWidth (2)
    }

    // clear background on double click
    Component clear
    Component clearData {
        Ref rootRef (root)
        Ref contentRef (content)
    }
    NativeAction clearAction (clearContent, clearData, 1)
    clear -> clearAction
    DoubleClick doubleClick (bgRect, f)
    doubleClick.double_click -> clear

    // temp points to draw sketches
    APoint prevPoint (0,0)
    APoint currentPoint(0,0)
    APoint midPoint (0,0)
    APoint prevMidPoint (0,0)

    // set the origin on press
    Component onPressAction {
        f.press.x =: prevPoint.x
        f.press.y =: prevPoint.y
        f.press.x =: prevMidPoint.x
        f.press.y =: prevMidPoint.y
    }

    // sketching
    FSM sketchFSM {
        State idle
        State sketching {
            f.move.x => currentPoint.x
            f.move.y => currentPoint.y

            //data component with points and content
            Component actionData {
                Ref prevRef (prevPoint)
                Ref currentRef (currentPoint) 
                Ref midRef (midPoint) 
                Ref prevMidRef (prevMidPoint)         
                Ref contentRef (content)
                clearData.contentRef => contentRef
                Component placeHolder
                Ref tmpShapeRef (placeHolder)
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
        State noprompt 

        prompt -> noprompt (f.press)
        noprompt -> prompt (clear)
    }
}

run root
run syshook
