/*
*  djnn Smala compiler
*
*  The copyright holders for the contents of this file are:
*    Ecole Nationale de l'Aviation Civile, France (2020-2021)
*  See file "license.terms" for the rights and conditions
*  defined by copyright holders.
*
*
*  Contributors:
*    
*		Vincent Peyruqueou <vincent.peyruqueou@enac.fr>
*
*/

use core
use base
use display
use gui
use comms

import ShareCursorPosition
import RemoteCursor

_native_code_
%{
  #include <regex>
  #include <iostream>
  using namespace std;
%}


_action_
action_cursor(Process c)
%{
  // Get the source that triggered the native action:
  Process *src = c->get_activation_source ();
  string msg_cursor = toString(src);

  // Get the user_data
  Process *self = (Process*) get_native_user_data (c);

  regex rgx("Uid=([[:print:]]*) Color=(\\d*) X=(\\S*) Y=(\\S*)");
  smatch match;

  if (std::regex_match(msg_cursor, match, rgx))  
  {
    string remote_uid = match[1];
    int remote_color = stoi(match[2]);
    double remote_x = stod(match[3]);
    double remote_y = stod(match[4]);

    //cout << "Uid: " << remote_uid << " -- X: " << remote_x << " -- Y: " << remote_y << endl;

    Process* remote_cursors = self->find_child("remote_cursors");
    if (remote_cursors != nullptr)
    {
      Process* cursor = remote_cursors->find_child(remote_uid);
      if (cursor == nullptr)
      {
        cout << "cursor " << remote_uid << " doesn't exist...create it" << endl;
        /*auto* cursor =*/ RemoteCursor(remote_cursors, remote_uid, remote_uid, remote_x, remote_y, remote_color);
      }
      else
      {
        ((DoubleProperty*)cursor->find_child("x"))->set_value(remote_x, true);
        ((DoubleProperty*)cursor->find_child("y"))->set_value(remote_y, true);
        
        if (((IntProperty*)cursor->find_child("color"))->get_value() != remote_color) {
          ((IntProperty*)cursor->find_child("color"))->set_value(remote_color, true);
        }
      }
    }
    else {
      cerr << "remote_cursors doesn't exist" << endl;
    }
  }  
%}


_main_
Component root {
  
  // -------------- Parameters -------------- //
  // UID of our cursor in remote apps
  String uid ("ENAC LII")
  //String uid ("ATCO Tower")
  //String uid ("ATCO Ground")
  
  // Color of our cursor in remote apps
  //Int color (0) // Black
  //Int color (0xFF0000) // Red
  //Int color (0x00FF00) // Green
  Int color (255) // Int color (0x0000FF) // Blue
  
  String bus_ip_and_port ("192.168.1.255:2010")
  // -------------- Parameters -------------- //


  Frame frame ("Share Cursor - " + uid, 0, 0, 400, 400)
  mouseTracking = 1

  Exit exit (0, 1)
  frame.close -> exit

  TextPrinter tp
  "Mouse Cursor: UID = " + uid + " -- color = " + toString(color) + " -- IVY = " + bus_ip_and_port =: tp.input
  //print ("Mouse Cursor: UID = " + uid + " -- color = " + color + " -- IVY = " + bus_ip_and_port + "\n")


  // Decoration displayed in background
  Component background {
    FillColor _ (128, 128, 128)
    Rectangle r1 (50, 50, 150, 100, 0, 0)
    Rectangle r2 (250, 250, 100, 150, 0, 0)
  }

  // Remote cursors displayed in middle layer
  Component remote_cursors

  // Circle in the top right corner to show my color (displayed on foreground)
  Component my_color {
    FillColor _ ($color)
    Circle c ($frame.width, 0, 25)
    frame.width => c.cx
  }


  // ------- Log Printer to receive a Message in Terminal ---
  LogPrinter lp ("ivy bus in: ")

  IvyAccess ivybus (toString(bus_ip_and_port), "cursors", "READY")
  {
    //String regex_cursor_moved ("^CursorMoved Uid=([[:print:]]*) Color=(\\d*) X=(\\S*) Y=(\\S*)")
    String regex_cursor_moved ("^CursorMoved (.*)")

    String regex_cursor_paused ("^CursorPaused Uid=([[:print:]]*)")
  }

  // Create connectors
  String msg_cursor ("")
  ivybus.in.regex_cursor_moved.[1] => msg_cursor

  String cursor_uid_paused ("")
  ivybus.in.regex_cursor_paused.[1] => cursor_uid_paused


  NativeAction na_cursor (action_cursor, root, 1)
  msg_cursor -> na_cursor

  cursor_uid_paused -> na_cursor_uid_paused: (root) {
    cursor = find(&root.remote_cursors, toString(root.cursor_uid_paused))
    if (&cursor != null) {
      notify cursor.pause
    }
    else {
      print ("Unknown Uid = " + root.cursor_uid_paused + "\n")
    }
  }


  // Allows to share our cursor position (every 250ms) with others apps
  ShareCursorPosition share (frame, ivybus, toString(uid), $color, 250)

}