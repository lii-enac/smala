/*
*  djnn Smala compiler
*
*  The copyright holders for the contents of this file are:
*    Ecole Nationale de l'Aviation Civile, France (2017)
*  See file "license.terms" for the rights and conditions
*  defined by copyright holders.
*
*
*  Contributors:
*    Nicolas Saporito  <nicolas.saporito@enac.fr>
*
*/

use core
use base
use display
use gui

import PanAndZoom
import PanAndZoomGrid

_native_code_
%{
#include <math.h>

inline
double myfract(double f)
{
  return f - floor(f);
}
%}

_main_
Component root {

  Frame frame ("Grid Zoom", 0, 0, 1024, 1024)
  Exit ex (0, 1)
  frame.close -> ex

  // Tile Grid layer
  Component _ {
    Component grid {
      PanAndZoomGrid pzg (frame.move, frame.press, frame.release, frame.wheel)

      Scaling zoom_tr (1,1, 0,0)
      Translation pan_tr (0,0)

      pzg.zoom =:> zoom_tr.sx, zoom_tr.sy
      pzg.{xpan, ypan} =:> pan_tr.{tx, ty}
      TextPrinter tp
      //pzg.zoom_level =:> tp.input
      
      NoFill _
      
      OutlineColor _ (255, 255, 255)

      int fwidth = 1024
      int fheight = 1024
      int num_col = 2 + fwidth/256 // 1 more left, 1 more right
      int num_row = 2 + fheight/256 // 1 more up, 1 more bottom

      // layer of 'large' red squares, should be the lowest definition tiles
      Component _ {
        OutlineWidth ow(1)
        1.0 / pzg.zoom =:> ow.width
        //ow.width =:> tp.input
        OutlineOpacity o(1.0)
        FillColor oc(90,0,0)
        NoOutline _
        //(1.0-myfract($pzg.zoom)) =:> o.a
        // pzg.zoom_level % 2 ? 255 : 0   =:> oc.r
        // pzg.zoom_level % 2 ?   0 : 255 =:> oc.g
        int offset = 8
        for (int col=0; col<num_col; col++) {
          for (int row=0; row<num_row; row++) {
            Rectangle rectangle (-256+col*256+offset, -256+row*256+offset, 255-offset*2, 255-offset*2, 3, 3)
            //Rectangle rectangle (-256+col*256, -256+row*256, 255, 255, 3, 3)
          }
        }
      }

      // layer of 'smaller' green squares .i.e should be the highest definition tiles, those that are supposed to appear when zooming in
      Component _ {
        OutlineWidth ow(1)
        1.0 / pzg.zoom =:> ow.width
        OutlineOpacity o(1.0)
        OutlineColor oc(0,255,0)
        //myfract($pzg.zoom) =:> o.a
        // pzg.zoom_level % 2 ? 255 : 0   =:> oc.g
        // pzg.zoom_level % 2 ?   0 : 255 =:> oc.r

        for (int col=0; col<num_col*2; col++) {
          for (int row=0; row<num_row*2; row++) {
            //Rectangle rectangle (-256+col*128+4, -256+row*128+4, 127-8, 127-8, 3, 3)
            Rectangle rectangle (-256+col*128, -256+row*128, 127, 127, 3, 3)
          }
        }
      }
    }
    FillColor _(255,255,255)
    // Text map_col_txt(20,20,"col")
    // Text map_row_txt(50,20,"row")
    Text _ (20,20, "zoom level: ")
    Text zlt (100,20, "--")
    grid.pzg.zoom_level =:> zlt.text


    // manage notification of change in the grid (new zoom level, new xgrid, new ygrid)
    Int prev_zoom_level(0)
    Int prev_xgrid(0)
    Int prev_ygrid(0)


    // the following pzg's properties are changed only when they differ from the previous version
    
    grid.pzg.zoom_level =: prev_zoom_level // init prev
    grid.pzg.xgrid =: prev_xgrid // init prev
    grid.pzg.ygrid =: prev_ygrid // init prev
    
    TextPrinter tp
  
    grid.pzg.zoom_level -> {
      "zoom_level changed: from " + toString(prev_zoom_level) + " to " + toString(grid.pzg.zoom_level) =: tp.input
      grid.pzg.zoom_level =: prev_zoom_level // update prev
    }
    grid.pzg.xgrid -> {
      "xgrid      changed: from " + toString(prev_xgrid) + " to " + toString(grid.pzg.xgrid) =: tp.input
      
      // here should translate
      grid.pzg.xpan % 256 =: grid.pzg.xpan
      // and change image
      // ...

      grid.pzg.xgrid =: prev_xgrid // update prev
    }
    grid.pzg.ygrid -> {
      "ygrid      changed: from " + toString(prev_ygrid) + " to " + toString(grid.pzg.ygrid) =: tp.input
      // here should translate
      grid.pzg.ypan % 256 =: grid.pzg.ypan
      // and change image
      // ...

      grid.pzg.ygrid =: prev_ygrid // update prev
    }
  }

  // Overlaid graphics
  Component _ {
    PanAndZoom pz (frame.move, frame.press, frame.release, frame.wheel)

    Scaling zoom_tr (1,1, 0,0)
    Translation pan_tr (0,0)
    
    pz.zoom =:> zoom_tr.sx, zoom_tr.sy
    pz.{xpan, ypan} =:> pan_tr.{tx, ty}

    // Graphics
    FillColor _ (70, 70, 70)
    Text ts (200, 200, "The scene can be zoomed in and out with the mouse wheel")

    FillColor _ (200, 200, 200)
    FillOpacity _ (0.5)
    OutlineColor _ (70, 70, 70)
    Rectangle rectangle (100, 100, 100, 100, 0, 0)
  }

  

}

