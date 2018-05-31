use core
use base
use gui

import Slider

_action_
upraise_action (Component c)
%{
   Process *toMove = (Process*) get_native_user_data (c);
   Process *parent = toMove->get_parent ();
   parent->remove_child (toMove);
   parent->add_child (toMove, "");
 %}

_action_
reset_pos_action (Component c)
%{
   Process *toMove = (Process*) get_native_user_data (c);
   int i = ((IntProperty*) toMove->find_component ("_index"))->get_value ();
   int last = ((IntProperty*) toMove->find_component ("last"))->get_value ();
   if (i == last)
    return;
   List *parent = (List*) toMove->get_parent ();
   parent->remove_child (toMove);
   char spec[16];
   sprintf (spec, ">%d", i);
   parent->insert (toMove, spec);
%}

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
  char* path =  (char*) malloc (sz * sizeof (char));
  sprintf (path, "file://%s/%s", prefix, file);
  path[sz-1] = '\0';
  free (prefix);
  return path;
}
%}

 _define_
 Tab (Component frame, Component tabManager, string _label, int index) {
  FillColor foreground_color (250, 250, 250)
  string path = CCall (buildPath, "img/tab.svg")
  g_tab = loadFromXML (path)
 
  Int _index (index)
  Int last (-1)

  Component label_box {
    Translation pos  (index * 160, 0)
    tab << g_tab.layer1.tab
    RectangleClip clip (0, 0, 150, 25)
    FillColor font_color (0, 0, 0)
    FontSize fs (0, 11)
    Text label (25, 22, _label)
  }
  label aka label_box.label.text

  Component pane {
    FillOpacity opacity (1)
  }

  Spike setOnTop
  Spike selected
  Spike unselect
  setLast = _index =: last : 1
  setOnTop -> setLast

  NativeAction upraise (upraise_action, this, 1)
  NativeAction reset_pos (reset_pos_action, this, 1)

  FSM fsm {
    State st_unselected {
      tabManager.unselected_color =: foreground_color.r, foreground_color.g, foreground_color.b, label_box.tab.fill.r, label_box.tab.fill.g, label_box.tab.fill.b
      tabManager.unselected_font_color =: label_box.font_color.r, label_box.font_color.g, label_box.font_color.b
      tabManager.unselected_border_color =: label_box.tab.stroke.r, label_box.tab.stroke.g, label_box.tab.stroke.b
	  tabManager.unselectAll->reset_pos
    }
    State st_selected {
      tabManager.selected_color =: foreground_color.r, foreground_color.g, foreground_color.b,label_box.tab.fill.r, label_box.tab.fill.g, label_box.tab.fill.b
      tabManager.selected_font_color =: label_box.font_color.r, label_box.font_color.g, label_box.font_color.b
      OutlineWidth ow (1)
      OutlineColor border_color (100, 100, 100)
      tabManager.selected_border_color =: border_color.r, border_color.g, border_color.b, label_box.tab.stroke.r, label_box.tab.stroke.g, label_box.tab.stroke.b
      Line left (0, 22, 50, 22)
      Line right (50, 22, 300, 22)
      label_box.pos.tx => left.x2
      g_tab.layer1.left_pos.y => left.y1, left.y2, right.y1, right.y2
      label_box.pos.tx +  g_tab.layer1.right_pos.x => right.x1
      frame.width => right.x2

      Activator deselect (tabManager.unselectAll)
      tabManager.unselectAll->unselect
      Slider s (frame, 300, 50)
      frame.width - s.width - 30 => s.x
      frame.height - 30 => s.y

      s.output => pane.opacity.a
    }
	st_unselected->st_selected (label_box.tab.press, upraise)
	st_selected->st_unselected (unselect, reset_pos)
    st_unselected->st_selected (setOnTop)
  }
}