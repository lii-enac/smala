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
  #include <iostream>
  using namespace std;

  string str_ivy_bus = "192.168.1.255:2010";
  string str_uid = "ENAC LII";
  string str_color = "0x000000";

  char* getCmdOption(char ** begin, char ** end, const std::string & option)
  {
      char ** itr = std::find(begin, end, option);
      if (itr != end && ++itr != end)
      {
          return *itr;
      }
      return 0;
  }

  int init (int argc, char * argv[])
  {
    char* ivybus = getCmdOption(argv, argv + argc, "-b");
    if (ivybus) {
        str_ivy_bus = string (ivybus);
        cout << "using ivy bus from cmd line '" << str_ivy_bus << "'" << endl;
    }
    else
        cout << "using default ivy bus '" << str_ivy_bus << "'" << endl;

    char* uid = getCmdOption(argv, argv + argc, "-u");
    if (uid) {
        str_uid = string (uid);
        cout << "using uid from cmd line '" << str_uid << "'" << endl;
    }
    else
        cout << "using default name '" << str_uid << "'" << endl;

    char* color = getCmdOption(argv, argv + argc, "-c");
    if (color) {
        str_color = string (color);
        cout << "using color from cmd line '" << str_color << "'" << endl;
    }
    else
        cout << "using default color '" << str_color << "'" << endl;

    return 0;
  }

%}

_action_
action_cursor (Process src, Process self)
{
  ivy = find (self, "ivybus")
  if (&ivy != null)
  {
    string remote_uid = getString (ivy.in.rgx_cursor_moved.[1])
    int remote_color = stoi (getString (ivy.in.rgx_cursor_moved.[2]))
    double remote_x = stod (getString (ivy.in.rgx_cursor_moved.[3]))
    double remote_y = stod (getString (ivy.in.rgx_cursor_moved.[4]))

    cursor = find (self, "remote_cursors/" + remote_uid)
    if (&cursor == null) {
      print ("Remote cursor '" + ivy.in.rgx_cursor_moved.[1] + "' does NOT exist...create it")
      remote_cursors = find (self, "remote_cursors")
      RemoteCursor (remote_cursors, remote_uid, remote_uid, remote_x, remote_y, remote_color)
    }
    else {
      //print ("Remote cursor '" + ivy.in.rgx_cursor_moved.[1] + "' EXIST...update it")
      //if (getInt (cursor.color) != remote_color) {
      // if (getInt (cursor.fill_c.value) != remote_color) {
      //   cursor.color = remote_color
      // }
      cursor.x = remote_x
      cursor.y = remote_y
    }
  }
}


_main_
Component root {
  init (argc, argv)

  // -------------- Parameters -------------- //
  String bus_ip_and_port (str_ivy_bus)
  String uid (str_uid)
  Int color (stoi (str_color, 0, 16))
  // -------------- Parameters -------------- //


  Frame frame ("Share Cursor - " + uid + " - " + bus_ip_and_port, 0, 0, 400, 400)
  mouseTracking = 1

  Exit exit (0, 1)
  frame.close -> exit

  TextPrinter tp
  "Mouse Cursor: UID = '" + uid + "' -- color = " + toString(color) + " -- IVY = " + bus_ip_and_port =: tp.input
  //print ("Mouse Cursor: UID = '" + uid + "' -- color = " + color + " -- IVY = " + bus_ip_and_port)


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

  IvyAccess ivybus (getString(bus_ip_and_port), "cursors", "READY")
  {
    String rgx_cursor_moved ("^CursorMoved Uid=([[:print:]]*) Color=(\\d*) X=(\\S*) Y=(\\S*)")

    String rgx_cursor_paused ("^CursorPaused Uid=([[:print:]]*)")
  }

  // Create connectors
  NativeAction na_cursor (action_cursor, root, 1)
  ivybus.in.rgx_cursor_moved.[1] -> na_cursor

  String cursor_uid_paused ("")
  ivybus.in.rgx_cursor_paused.[1] => cursor_uid_paused

  cursor_uid_paused -> na_cursor_uid_paused: (root) {
    cursor = find(&root.remote_cursors, getString(root.cursor_uid_paused))
    if (&cursor != null) {
      notify cursor.pause
    }
    else {
      print ("Unknown Uid = '" + root.cursor_uid_paused + "'\n")
    }
  }


  // Allows to share our cursor position (every 200ms) with others apps
  ShareCursorPosition share (frame, ivybus, getString(uid), $color, 200)

}