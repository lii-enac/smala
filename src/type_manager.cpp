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

#include "type_manager.h"

#include <iostream>

namespace Smala {


  TypeManager::TypeManager ()
  {
  }

  TypeManager::~TypeManager ()
  {
  }

  void
  TypeManager::init ()
  {
    build_table (djnCoreGlobalSymTable, coreGlobalSymbols);
    build_table (djnDisplayGlobalSymTable, displayGlobalSymbols);
    build_table (djnInputGlobalSymTable, inputGlobalSymbols);
    build_table (djnPhidgetsGlobalSymTable, phidgetsGlobalSymbols);
    build_table (djnPowerGlobalSymTable, powerGlobalSymbols);
    build_table (djnGUIGlobalSymTable, guiGlobalSymbols);
    build_table (djnSoundGlobalSymTable, soundGlobalSymbols);

    build_table (animation, animation_types);
    build_table (base, base_types);
    build_table (core, core_types);
    build_table (comms, comms_types);
    build_table (display, display_types);
    build_table (files, files_types);
    build_table (gestures, gestures_types);
    build_table (gui, gui_types);
    build_table (input, input_types);
    build_table (macbook, macbook_types);
    build_table (modules, modules_types);
    build_table (network, network_types);
    build_table (objects, objects_types);
    build_table (phidgets, phidgets_types);
    build_table (power, power_types);
    build_table (qtwidgets, qtwidgets_types);
    build_table (sound, sound_types);

    m_types_tables.insert (std::pair<std::string, symtable_t>("Animation", animation));
    m_types_tables.insert (std::pair<std::string, symtable_t>("Base", base));
    m_types_tables.insert (std::pair<std::string, symtable_t>("Core", core));
    m_types_tables.insert (std::pair<std::string, symtable_t>("Comms", comms));
    m_types_tables.insert (std::pair<std::string, symtable_t>("Display", display));
    m_types_tables.insert (std::pair<std::string, symtable_t>("Files", files));
    m_types_tables.insert (std::pair<std::string, symtable_t>("Gestures", gestures));
    m_types_tables.insert (std::pair<std::string, symtable_t>("Gui", gui));
    m_types_tables.insert (std::pair<std::string, symtable_t>("Input", input));
    m_types_tables.insert (std::pair<std::string, symtable_t>("Macbook", macbook));
    m_types_tables.insert (std::pair<std::string, symtable_t>("Modules", modules));
    m_types_tables.insert (std::pair<std::string, symtable_t>("Network", network));
    m_types_tables.insert (std::pair<std::string, symtable_t>("Objects", objects));
    m_types_tables.insert (std::pair<std::string, symtable_t>("Phidgets", phidgets));
    m_types_tables.insert (std::pair<std::string, symtable_t>("Power", power));
    m_types_tables.insert (std::pair<std::string, symtable_t>("Qtwidgets", qtwidgets));
    m_types_tables.insert (std::pair<std::string, symtable_t>("Sound", sound));
  }

  void
  TypeManager::add_global_symbols (const std::string &module, symtable_t *table)
  {
    symtable_t symtable;
    if (module.compare ("Core") == 0) {
      symtable = djnCoreGlobalSymTable;
    }
    else if (module.compare ("Input") == 0) {
      symtable = djnInputGlobalSymTable;
    }
    else if (module.compare ("Display") == 0) {
      symtable = djnDisplayGlobalSymTable;
    }
    else if (module.compare ("Gui") == 0) {
      symtable = djnGUIGlobalSymTable;
    }
    else if (module.compare ("Phidgets") == 0) {
      symtable = djnPhidgetsGlobalSymTable;
    }
    else if (module.compare ("Power") == 0) {
      symtable = djnPowerGlobalSymTable;
    }
    else if (module.compare ("Sound") == 0) {
      symtable = djnSoundGlobalSymTable;
    }
    else {
      return;
    }
    table->insert (symtable.begin (), symtable.end ());
  }

  void
  TypeManager::init_types (const std::string &module, symtable_t &table)
  {
    symtable_t types_table;
    std::map<std::string, symtable_t>::const_iterator it;
    it = m_types_tables.find (module);
    if (it == m_types_tables.end ()) {
      std::cout << "Warning: unknown module: " << module << std::endl;
      return;
    } else {
      types_table = it->second;
      table.insert (types_table.begin (), types_table.end ());
    }
  }

  void
  TypeManager::build_table (symtable_t &table, type_entry* types)
  {
    int i = 0;
    while (types[i].type.compare ("0") != 0) {
      table.insert (
          std::pair<std::string, std::string> (types[i].type,
                                               types[i].djnName));
      i++;
    }
  }

}
