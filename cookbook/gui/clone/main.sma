/*
 *  djnn Smala compiler
 *
 *  The copyright holders for the contents of this file are:
 *    Ecole Nationale de l'Aviation Civile, France (2018)
 *  See file "license.terms" for the rights and conditions
 *  defined by copyright holders.
 *
 *
 *  Contributors:
 *    Mathieu Poirier <mathieu.poirier@enac.fr>
 *
 */

use core
use base
use display
use gui

_native_code_
%{
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

char*
buildPath (const char* file)
{
  char* prefix = getcwd (NULL, 0);
  int sz = strlen (prefix) + strlen (file) + 9;
  char* path = (char*) malloc (sz * sizeof (char));
  sprintf (path, "file://%s/%s", prefix, file);
  path[sz-1] = '\0';
  free (prefix);
  return path;
}
%}


 _main_
 Component root
 {

  Frame f ("clone", 0, 0, 600, 600)

  Group p1 {
    Rectangle rect1 (100, 100, 100, 100, 0, 0)
    Circle cric1 (300, 100, 50)
  }

  /* clone the all group */
  Translation t1 (0, 200)
  dolly1 << clone (p1)

  /* clone only circle */
  Translation t2 (0, 200)
  FillColor  _ (255, 0, 0)
  dolly2 << clone (dolly1.cric1)

  /* test with SVG */
  svg = loadFromXML (buildPath ("img/aircraft.svg"))

  Translation t3 (300, -300)
  Group aircraft {  
    background << svg.background
    label << svg.label
  }

  Translation t4 (0, 200)
  aircraft_clone << clone (aircraft)
  aircraft_clone.label.text = "CLONE" 

}

