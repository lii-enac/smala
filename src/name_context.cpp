/*
 *  djnn Smala compiler
 *
 *  The copyright holders for the contents of this file are:
 *    Ecole Nationale de l'Aviation Civile, France (2020)
 *  See file "license.terms" for the rights and conditions
 *  defined by copyright holders.
 *
 *
 *  Contributors:
 *    Mathieu Magnaudet <mathieu.magnaudet@enac.fr>
 *
 */

#include "name_context.h"

namespace Smala {
  SymTable::SymTable (SymTable* t) : _prev (t) {
    if (t == nullptr)
        init_symtable ();
  }

  void
  SymTable::init_symtable () {
    add_global_sym ("syshook", PROCESS);
    add_global_sym ("mainloop", PROCESS);
    add_global_sym ("InputDevices", PROCESS);
    add_global_sym ("Mice", PROCESS);
    add_global_sym ("TouchPanels", PROCESS);
    add_global_sym ("GPIOs", PROCESS);
    add_global_sym ("Styluses", PROCESS);
    add_global_sym ("RawPointers", PROCESS);
    add_global_sym ("Joysticks", PROCESS);
    add_global_sym ("RotaryKnobs", PROCESS);
    add_global_sym ("Absolute3DDevices", PROCESS);
    add_global_sym ("MakeyMakeys", PROCESS);
    add_global_sym ("Keyboards", PROCESS);
    add_global_sym ("Phidgets", PROCESS);
    add_global_sym ("InterfaceKits", PROCESS);
    add_global_sym ("hidePointer", INT);
    add_global_sym ("fullScreen", INT);
    add_global_sym ("mouseTracking", INT);
    add_global_sym ("DrawingRefreshManager", PROCESS);
    add_global_sym ("GenericMouse", PROCESS);
    add_global_sym ("DefaultAudioListener", PROCESS);

    /* HTML Color Name */
    add_global_sym ("Black",INT);
    add_global_sym ("Navy",INT);
    add_global_sym ("DarkBlue",INT);
    add_global_sym ("MediumBlue",INT);
    add_global_sym ("Blue",INT);
    add_global_sym ("DarkGreen",INT);
    add_global_sym ("Green",INT);
    add_global_sym ("Teal",INT);
    add_global_sym ("DarkCyan",INT);
    add_global_sym ("DeepSkyBlue",INT);
    add_global_sym ("DarkTurquoise",INT);
    add_global_sym ("MediumSpringGreen",INT);
    add_global_sym ("Lime",INT);
    add_global_sym ("SpringGreen",INT);
    add_global_sym ("Aqua",INT);
    add_global_sym ("Cyan",INT);
    add_global_sym ("MidnightBlue",INT);
    add_global_sym ("DodgerBlue",INT);
    add_global_sym ("LightSeaGreen",INT);
    add_global_sym ("ForestGreen",INT);
    add_global_sym ("SeaGreen",INT);
    add_global_sym ("DarkSlateGray",INT);
    add_global_sym ("DarkSlateGrey",INT);
    add_global_sym ("LimeGreen",INT);
    add_global_sym ("MediumSeaGreen",INT);
    add_global_sym ("Turquoise",INT);
    add_global_sym ("RoyalBlue",INT);
    add_global_sym ("SteelBlue",INT);
    add_global_sym ("DarkSlateBlue",INT);
    add_global_sym ("MediumTurquoise",INT);
    add_global_sym ("Indigo",INT);
    add_global_sym ("DarkOliveGreen",INT);
    add_global_sym ("CadetBlue",INT);
    add_global_sym ("CornflowerBlue",INT);
    add_global_sym ("RebeccaPurple",INT);
    add_global_sym ("MediumAquaMarine",INT);
    add_global_sym ("DimGray",INT);
    add_global_sym ("DimGrey",INT);
    add_global_sym ("SlateBlue",INT);
    add_global_sym ("OliveDrab",INT);
    add_global_sym ("SlateGray",INT);
    add_global_sym ("SlateGrey",INT);
    add_global_sym ("LightSlateGray",INT);
    add_global_sym ("LightSlateGrey",INT);
    add_global_sym ("MediumSlateBlue",INT);
    add_global_sym ("LawnGreen",INT);
    add_global_sym ("Chartreuse",INT);
    add_global_sym ("Aquamarine",INT);
    add_global_sym ("Maroon",INT);
    add_global_sym ("Purple",INT);
    add_global_sym ("Olive",INT);
    add_global_sym ("Gray",INT);
    add_global_sym ("Grey",INT);
    add_global_sym ("SkyBlue",INT);
    add_global_sym ("LightSkyBlue",INT);
    add_global_sym ("BlueViolet",INT);
    add_global_sym ("DarkRed",INT);
    add_global_sym ("DarkMagenta",INT);
    add_global_sym ("SaddleBrown",INT);
    add_global_sym ("DarkSeaGreen",INT);
    add_global_sym ("LightGreen",INT);
    add_global_sym ("MediumPurple",INT);
    add_global_sym ("DarkViolet",INT);
    add_global_sym ("PaleGreen",INT);
    add_global_sym ("DarkOrchid",INT);
    add_global_sym ("YellowGreen",INT);
    add_global_sym ("Sienna",INT);
    add_global_sym ("Brown",INT);
    add_global_sym ("DarkGray",INT);
    add_global_sym ("DarkGrey",INT);
    add_global_sym ("LightBlue",INT);
    add_global_sym ("GreenYellow",INT);
    add_global_sym ("PaleTurquoise",INT);
    add_global_sym ("LightSteelBlue",INT);
    add_global_sym ("PowderBlue",INT);
    add_global_sym ("FireBrick",INT);
    add_global_sym ("DarkGoldenRod",INT);
    add_global_sym ("MediumOrchid",INT);
    add_global_sym ("RosyBrown",INT);
    add_global_sym ("DarkKhaki",INT);
    add_global_sym ("Silver",INT);
    add_global_sym ("MediumVioletRed",INT);
    add_global_sym ("IndianRed",INT);
    add_global_sym ("Peru",INT);
    add_global_sym ("Chocolate",INT);
    add_global_sym ("Tan",INT);
    add_global_sym ("LightGray",INT);
    add_global_sym ("LightGrey",INT);
    add_global_sym ("Thistle",INT);
    add_global_sym ("Orchid",INT);
    add_global_sym ("GoldenRod",INT);
    add_global_sym ("PaleVioletRed",INT);
    add_global_sym ("Crimson",INT);
    add_global_sym ("Gainsboro",INT);
    add_global_sym ("Plum",INT);
    add_global_sym ("BurlyWood",INT);
    add_global_sym ("LightCyan",INT);
    add_global_sym ("Lavender",INT);
    add_global_sym ("DarkSalmon",INT);
    add_global_sym ("Violet",INT);
    add_global_sym ("PaleGoldenRod",INT);
    add_global_sym ("LightCoral",INT);
    add_global_sym ("Khaki",INT);
    add_global_sym ("AliceBlue",INT);
    add_global_sym ("HoneyDew",INT);
    add_global_sym ("Azure",INT);
    add_global_sym ("SandyBrown",INT);
    add_global_sym ("Wheat",INT);
    add_global_sym ("Beige",INT);
    add_global_sym ("WhiteSmoke",INT);
    add_global_sym ("MintCream",INT);
    add_global_sym ("GhostWhite",INT);
    add_global_sym ("Salmon",INT);
    add_global_sym ("AntiqueWhite",INT);
    add_global_sym ("Linen",INT);
    add_global_sym ("LightGoldenRodYellow",INT);
    add_global_sym ("OldLace",INT);
    add_global_sym ("Red",INT);
    add_global_sym ("Fuchsia",INT);
    add_global_sym ("Magenta",INT);
    add_global_sym ("DeepPink",INT);
    add_global_sym ("OrangeRed",INT);
    add_global_sym ("Tomato",INT);
    add_global_sym ("HotPink",INT);
    add_global_sym ("Coral",INT);
    add_global_sym ("DarkOrange",INT);
    add_global_sym ("LightSalmon",INT);
    add_global_sym ("Orange",INT);
    add_global_sym ("LightPink",INT);
    add_global_sym ("Pink",INT);
    add_global_sym ("Gold",INT);
    add_global_sym ("PeachPuff",INT);
    add_global_sym ("NavajoWhite",INT);
    add_global_sym ("Moccasin",INT);
    add_global_sym ("Bisque",INT);
    add_global_sym ("MistyRose",INT);
    add_global_sym ("BlanchedAlmond",INT);
    add_global_sym ("PapayaWhip",INT);
    add_global_sym ("LavenderBlush",INT);
    add_global_sym ("SeaShell",INT);
    add_global_sym ("Cornsilk",INT);
    add_global_sym ("LemonChiffon",INT);
    add_global_sym ("FloralWhite",INT);
    add_global_sym ("Snow",INT);
    add_global_sym ("Yellow",INT);
    add_global_sym ("LightYellow",INT);
    add_global_sym ("Ivory",INT);
    add_global_sym ("White",INT);
  }


  void
  SymTable::add_global_sym (const std::string& key, smala_t type) {
    _t.insert (std::pair<std::string, SmalaType> (key, type));
  }

  int
  SymTable::add_sym (const location& loc, const std::string& key, smala_t type) {
    std::map<std::string, SmalaType>::const_iterator it;
    it = _t.find (key);
    if (it == _t.end ()) {
      _t.insert (std::pair<std::string, SmalaType> (key, type));
      return 0;
    } else {
      std::cerr << "\nWarning duplicated symbol: " << key << " " << loc << "\n\n";
      return 1;
    }
  }

  bool
  SymTable::exists (const std::string &key) const {
    std::map<std::string, SmalaType>::const_iterator it;
    it = _t.find (key);
    if (it == _t.end ()) {
      if (_prev)
        return _prev->exists (key);
      else
        return 0;
    } else
      return 1;
  }

  SmalaType
  SymTable::get_type (const std::string &key) const {
    std::map<std::string, SmalaType>::const_iterator it;
    it = _t.find (key);
    if (it == _t.end ()) {
      if (_prev)
        return _prev->get_type (key);
      return UNDEFINED;
    } else {
      return it->second;
    }
  }
}
