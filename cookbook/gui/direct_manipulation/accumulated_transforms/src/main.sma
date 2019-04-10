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

import DemoLeft
import DemoRight

_main_
Component root {
	Exit ex (0, 1)
	mouseTracking = 1

	// -------------------------------
	// - Frame for left product demo - 
	// -------------------------------
	Frame frameLeft ("Accumulated transformations (left product)", 0, 0, 900, 1000)
	frameLeft.close -> ex

	// A few arbitrary transforms
	// (the tests below use local coordinates, so one can transform
	// the scene at will without impacting the tested interactions)
	Rotation _ (22.5, 0, 0)
	Translation _ (123, 24)
	Scaling _ (1.2, 1.2, 0, 0)
	
	// Graphic styles
	FillColor _ (200, 200, 200)
    OutlineColor _ (70, 70, 70)
    OutlineWidth _ (0)

    // Demo left product
    DemoLeft _ (frameLeft)


    // --------------------------------
	// - Frame for right product demo - 
	// --------------------------------
	Frame frameRight ("Accumulated transformations (right product)", 1000, 0, 900, 1000)
	frameRight.close -> ex

	// the arbitrary transforms
	Rotation _ (22.5, 0, 0)
	Translation _ (123, 24)
	Scaling _ (1.2, 1.2, 0, 0)
	
	// Graphic styles
	FillColor _ (200, 200, 200)
    OutlineColor _ (70, 70, 70)
    OutlineWidth _ (0)

    // Demo right product
    DemoRight _ (frameRight)
}

run root
run syshook
