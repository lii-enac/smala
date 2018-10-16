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
 *    Mathieu Magnaudet <mathieu.magnaudet@enac.fr>
 *    Stephane Conversy <stephane.conversy@enac.fr>
 *
 */

#include "cpp_type_manager.h"

#include <iostream>

namespace Smala {


  CPPTypeManager::CPPTypeManager () : TypeManager ()
  {
    type_entry coreGS[] = {
      {"syshook", "system_hook"},
      {"0", "0"}
    };
    coreGlobalSymbols = coreGS;

    type_entry displayGS[] = {
//      {"Displays", "djnDisplays"},
      {"0", "0"}
    };
    displayGlobalSymbols = displayGS;

    type_entry inputGS[] = {
      {"InputDevices", "djnInputDevices"},
      {"Mice", "djnMice"},
      {"TouchPanels", "djnTouchPanels"},
#if 0
      {"Styluses", "djnStyluses"},
      {"RawPointers", "djnRawPointers"},
      {"Joysticks", "djnJoysticks"},
      {"RotaryKnobs", "djnRotaryKnobs"},
      {"Absolute3DDevices", "djnAbsolute3DDevices"},
      {"MakeyMakeys", "djnMakeyMakeys"},
      {"Keyboards", "djnKeyboards"},
#endif
      {"0", "0"}
    };
    inputGlobalSymbols = inputGS;

    type_entry phidgetsGS[] = {
//      {"InterfaceKits", "djnInterfaceKits"},
      {"0", "0"}
    };
    phidgetsGlobalSymbols = phidgetsGS;

    type_entry powerGS[] = {
 //     {"Batteries", "djnBatteries"},
      {"0", "0"}
    };
    powerGlobalSymbols = powerGS;

    type_entry guiGS[] = {
      {"fullScreen", "full_screen"},
      {"mouseTracking", "mouse_tracking"},
      {"0", "0"}
    };
    guiGlobalSymbols = guiGS;

    type_entry soundGS[] = {
      //{"PortAudio", "djnPortAudio"},
      {"0", "0"}
    };
    soundGlobalSymbols = soundGS;

    type_entry animation_t[] = {
      {"0", "0"}
    };
    animation_types = animation_t;


    type_entry base_t[] = {
      {"0", "0"}
    };
    base_types = base_t;

    type_entry comms_t[] = {
      {"0", "0"}
    };
    comms_types = comms_t;

    type_entry core_t[] = {
      {"Alias", "alias"},
      {"Blank", "Spike"},
      {"Bool", "BoolProperty"},
      {"Double", "DoubleProperty"},
      {"Int", "IntProperty"},
      {"Ref", "RefProperty"},
      {"setBool", "set_value"},
      {"setDouble", "set_value"},
      {"setInt", "set_value"},
      {"SetIterator", "SetIterator"},
      {"setRef", "set_value"},
      {"setString", "set_value"},
      {"String", "TextProperty"},
      {"0", "0"}
    };
    core_types = core_t;

    type_entry display_t[] = {
      {"0", "0"}
    };
    display_types = display_t;

    type_entry files_t[] = {
      {"0", "0"}
    };
    files_types = files_t;

    type_entry gestures_t[] = {
      {"0", "0"}
    };
    gestures_types = gestures_t;

    type_entry gui_t[] = {
      {"Frame", "Window"},
      {"0", "0"}
    };
    gui_types = gui_t;

    type_entry input_t[] = {
      {"0", "0"}
    };
    input_types = input_t;

    type_entry macbook_t[] = {
      {"0", "0"}
    };
    macbook_types = macbook_t;

    type_entry modules_t[] = {
      {"0", "0"}
    };
    modules_types = modules_t;

    type_entry network_t[] = {
      {"0", "0"}
    };
    network_types = network_t;

    type_entry objects_t[] = {
      {"0", "0"}
    };
    objects_types = objects_t;

    type_entry phidgets_t[] = {
      {"0", "0"}
    };
    phidgets_types = phidgets_t;

    type_entry power_t[] = {
      {"0", "0"}
    };
    power_types = power_t;

    type_entry qtwidgets_t[] = {
      {"0", "0"}
    };
    qtwidgets_types = qtwidgets_t;

    type_entry sound_t[] = {
      {"0", "0"}
    };
    sound_types = sound_t;

    init ();
  }

  CPPTypeManager::~CPPTypeManager ()
  {
  }

}
