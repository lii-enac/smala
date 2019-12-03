/*
 *  djnn Smala compiler
 *
 *  The copyright holders for the contents of this file are:
 *      Ecole Nationale de l'Aviation Civile, France (2018)
 *  See file "license.terms" for the rights and conditions
 *  defined by copyright holders.
 *
 *
 *  Contributors:
 *      Mathieu Magnaudet <mathieu.magnaudet@enac.fr>
 *
 */

#pragma once

#include <map>
#include <string>

namespace Smala {

typedef struct type_entry {
  std::string type;
  std::string djnName;
} type_entry;

class TypeManager {
public:
    typedef std::map <std::string, std::string> symtable_t;
	TypeManager ();
	~TypeManager ();
	void add_global_symbols (const std::string &module, symtable_t *table);
	void init_types (const std::string &module, symtable_t &table);
	void add_types (const std::string &module, symtable_t &table);
	std::string get_smala_symbol (const std::string& key);
	void init ();
protected:
	void build_table (symtable_t &table, type_entry* types);
	symtable_t animation;
	symtable_t base;
	symtable_t comms;
	symtable_t core;
	symtable_t display;
	symtable_t files;
	symtable_t gestures;
	symtable_t graphics;
	symtable_t gui;
	symtable_t input;
	symtable_t macbook;
	symtable_t modules;
	symtable_t network;
	symtable_t objects;
	symtable_t phidgets;
	symtable_t power;
	symtable_t qtwidgets;
	symtable_t sound;

	symtable_t djnCoreGlobalSymTable;
	symtable_t djnDisplayGlobalSymTable;
	symtable_t djnInputGlobalSymTable;
	symtable_t djnPhidgetsGlobalSymTable;
	symtable_t djnPowerGlobalSymTable;
	symtable_t djnGUIGlobalSymTable;
	symtable_t djnSoundGlobalSymTable;
  symtable_t smalaGlobalSymTable;

	type_entry *coreGlobalSymbols;
	type_entry *displayGlobalSymbols;
	type_entry *inputGlobalSymbols;
	type_entry *phidgetsGlobalSymbols;
	type_entry *powerGlobalSymbols;
	type_entry *guiGlobalSymbols;
	type_entry *soundGlobalSymbols;
	type_entry *animation_types;
	type_entry *base_types;
	type_entry *comms_types;
	type_entry *core_types;
	type_entry *display_types;
	type_entry *files_types;
	type_entry *gestures_types;
	type_entry *gui_types;
	type_entry *input_types;
	type_entry *macbook_types;
	type_entry *modules_types;
	type_entry *network_types;
	type_entry *objects_types;
	type_entry *phidgets_types;
	type_entry *power_types;
	type_entry *qtwidgets_types;
	type_entry *sound_types;
	type_entry *smala_symbols;

	std::map <std::string, symtable_t> m_types_tables;
};
}
