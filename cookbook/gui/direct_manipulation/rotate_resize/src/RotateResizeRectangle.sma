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

import Drag
import HandleN
import HandleNE
import HandleE
import HandleSE
import HandleS
import HandleSW
import HandleW
import HandleNW
import RotationHandle

_define_
RotateResizeRectangle (Process frame, double x_, double y_) {

    //// Translation to initially place the rectangle (use it just for that, 
    // the other transforms will be accumulated into the Homography named transforms, see below)
    Translation _ (x_, y_)

    //// Non visual rectangle serving as a local coordinates system to apply graphics transforms
    // In case of left multiplication the transforms must be set up in the ref coordinates system
    Rectangle ref (0, 0, 0, 0, 0, 0)

    //// Homography to accumulate graphics transforms
    Homography transforms

    //// Styles
    // Set outline width to 0 allows to have a constant outline width 
    // (default behavior in Qt implementation)
    OutlineWidth _ (0)

    //// Rectangle to resize and rotate
    // In case of right multiplication the transforms must be set up in the rect coordinates system
    Rectangle rect (0, 0, 100, 100, 0, 0)
    Double minWidth (20)
    Double minHeight (20)

    //// Rotation handle and center

    Line rCenterToHandleLine (50, 50, 0, 0)
    rect.width /2 :: rCenterToHandleLine.x2
    0 :: rCenterToHandleLine.y2

    Component rCenter {
        // Make this handle look size-invariant (inverse of the scalings accumulated on the rectangle)
        Scaling scaling_inv (1, 1, 50, 50)
        1 / transforms.accsx =:> scaling_inv.sx
        1 / transforms.accsy =:> scaling_inv.sy
        Circle center (50, 50, 1)
    }

    RotationHandle rHandle (frame, this, -20, 10)
    rect.width /2 =: rHandle.x
    0 =: rHandle.y

    //// Resizing handles

    HandleN handleN (this, 10)
    rect.width /2 =: handleN.x
    0 =: handleN.y

    HandleNE handleNE (this, 10)
    rect.width =: handleNE.x
    0 =: handleNE.y

    HandleE handleE (this, 10)
    rect.width =: handleE.x
    rect.height /2 =: handleE.y

    HandleSE handleSE (this, 10)
    rect.width =: handleSE.x
    rect.height =: handleSE.y

    HandleS handleS (this, 10)
    rect.width /2 =: handleS.x
    rect.height =: handleS.y

    HandleSW handleSW (this, 10)
    0 =: handleSW.x
    rect.height =: handleSW.y

    HandleW handleW (this, 10)
    0 =: handleW.x
    rect.height /2 =: handleW.y

    HandleNW handleNW (this, 10)
    0 =: handleNW.x
    0 =: handleNW.y

    //// Drag
    
    Drag drag (rect, transforms)
}
