use core
use base
use display
use gui

import LocalToLocal

/*
 * Rotation Handle
 * target must be a RotateResizeRectangle
 */
_define_
RotationHandle (Process frame, Process target, double distance, double size) {

    /* ----- Interface ----- */
    Double x (0)
    Double y (0)
    /* --- End Interface --- */

    //// Translation to initially place the rectangle
    Translation translation (0, 0)
    x =:> translation.tx
    y =:> translation.ty

    //// Aliases from the target
    rect aka target.rect
    ref aka target.ref
    transforms aka target.transforms
    rotationCenter aka target.rCenter.center

    //// Make this handle look size-invariant (inverse of the scalings accumulated on the target)
    Scaling scaling_inv (1, 1, 0, 0)
    1 / transforms.accsx =:> scaling_inv.sx
    1 / transforms.accsy =:> scaling_inv.sy

    //// Graphics
    Line _ (0, 0, 0, distance)
    Circle handle (0, distance, size)

    //// Rotation management
    // The rotation is around the center of the target (rect) 
    // and is done by left multiplying the homography (transforms) by a new rotation matrix, 
    // rather than right multiplying it (like it's done for the resizing handles).
    // Reason: 
    // When simply accumulating transforms on the same side (left or right product),
    // non homothetic scalings are skewed by ulterior rotations, so we must decompose 
    // the scalings, apply the new rotation, then recompose the scalings, which is more 
    // difficult than directly apply the rotation on the left with the good coordinates conversion:
    // right product -> local coordinates system of the node placed just after the Homography (rect)
    // left product  -> local coordinates system of the node placed just before the Homography (ref)
            
    // Store mouse press in rect coord system
    ScreenToLocal s2l_press (rect)
    handle.press.x =:> s2l_press.inX
    handle.press.y =:> s2l_press.inY

    FSM rotateFsm {
        State idle

        State pressed {
            // Set up rotation center
            LocalToLocal l2l_center (rect, ref)
            rotationCenter.cx =:> l2l_center.inX
            rotationCenter.cy =:> l2l_center.inY
            l2l_center.outX =:> transforms.leftRotateBy.cx
            l2l_center.outY =:> transforms.leftRotateBy.cy
        }

        State rotate {
            // Get mouse move in rect coord system
            ScreenToLocal s2l_move (rect)
            frame.move.x =:> s2l_move.inX
            frame.move.y =:> s2l_move.inY

            // Compute and apply rotation angle
            // (ArcTangent2 the vector going from rotation center to new mouse position)
            ArcTangent2 compute (0, 0)
            s2l_move.outX - s2l_press.outX =:> compute.x
            s2l_move.outY - s2l_press.outY + y - rotationCenter.cy =:> compute.y

            Double degrees (0)
            Double pi (3.14159265)

            // Convert to degrees and remove the -90 degrees offset of the initial angle
            (compute.result * 180 / pi) + 90 =:> transforms.leftRotateBy.da
        }

        idle -> pressed (handle.press)
        pressed -> idle (frame.release)
        pressed -> rotate (frame.move)
        rotate -> idle (frame.release)
    }
}
