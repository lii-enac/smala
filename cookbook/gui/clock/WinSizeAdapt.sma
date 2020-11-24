/*
 *	The copyright holders for the contents of this file are:
 *		Ecole Nationale de l'Aviation Civile, France (2018)
 *	All rights reserved.
 *
 *
 *	Contributors:
 *		Vinot Jean-Luc <jean-luc.vinot@enac.fr>
 *
 */

use core
use base
use gui

_define_
WinSizeAdapt (Process f, Process tr, Process sc)
{

	/* some geometric Int and Double Properties */
	Int initFW (800)
	Int initFH (1120)
	Double ratioW (1.0)
	Double ratioH (1.0)

	// record init frame size
	f.width =: initFW
	f.height =: initFH

	// compute width and height ratio (real/init size)
	f.width / initFW => ratioW
	f.height / initFH => ratioH

	Double newX (0)
	Double newY (0)
	Double minScale (1.0)

	// create connectors to adjust scene translation
	newX => tr.tx
	newY => tr.ty

	// update minScale with min of horizontal and vertical ratios
	ratioW < ratioH ? ratioW : ratioH => minScale
	minScale => sc.sx 
	minScale => sc.sy 

	// compute new scene translation according to the new scale
	(f.width - (initFW * minScale)) / 2 => newX
	(f.height - (initFH * minScale)) / 2 => newY


}
