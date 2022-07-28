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
      
      NoFill _
      
      OutlineColor _ (255, 255, 255)

      int fwidth = 1024
      int fheight = 1024
      int num_col = 2 + fwidth/256 // 1 more left, 1 more right
      int num_row = 2 + fheight/256 // 1 more up, 1 more bottom

      // layer of 'large' squares
      Component _ {
        OutlineWidth ow(2)
        2.0 / pzg.zoom =:> ow.width
        OutlineOpacity o(1.0)
        (1.0-myfract($pzg.zoom)) =:> o.a
        
        for (int col=0; col<num_col; col++) {
          for (int row=0; row<num_row; row++) {
            Rectangle rectangle (-256+col*256, -256+row*256, 255, 255, 0, 0)
          }
        }
      }

      // layer of 'smaller' squares .i.e those that are supposed to appear
      Component _ {
        OutlineWidth ow(1)
        1.0 / pzg.zoom =:> ow.width
        OutlineOpacity o(1.0)
        myfract($pzg.zoom) =:> o.a
        for (int col=0; col<num_col*2; col++) {
          for (int row=0; row<num_row*2; row++) {
            Rectangle rectangle (-256+col*128, -256+row*128, 127, 127, 0, 0)
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
  }

  // Overlay graphics
  Component _ {
    PanAndZoom pz (frame.move, frame.press, frame.release, frame.wheel)

    Scaling zoom_tr (1,1, 0,0)
    Translation pan_tr (0,0)
    
    pz.zoom =:> zoom_tr.sx, zoom_tr.sy
    pz.{xpan, ypan} =:> pan_tr.{tx, ty}

    // Graphics
    FillColor _ (70, 70, 70)
    Text ts (200, 200, "The scene can be zoomed by mouse wheel")

    FillColor _ (200, 200, 200)
    FillOpacity _ (0.5)
    OutlineColor _ (70, 70, 70)
    Rectangle rectangle (100, 100, 100, 100, 0, 0)
  }

  

}

