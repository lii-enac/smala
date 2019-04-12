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

#include "preamble.h"

#include <iostream>


using namespace Smala;

Preamble::Preamble () :
  m_node_list ()
{
}

Preamble::~Preamble()
{
}

void
Preamble::add_import (const std::string &import) {
  std::size_t pos = import.find ('.');
  std::string ns, name;
  if (pos == std::string::npos) {
    ns = import;
  }
  else {
    ns = import.substr (0, pos);
    name = import.substr (pos + 1, import.length () - 1);
  }
  Node *n = new Node ("", import);
  n->set_node_type (IMPORT);
  add_node (n);
  if (!name.empty ())
    m_import_list.push_back (name);
  else
    m_import_list.push_back (ns);
}

void
Preamble::add_use (const std::string &use)
{
  Node *n = new Node ("", use);
  n->set_node_type(USE);
  m_use_list.push_back (use);
  add_node (n);
}

std::vector<std::string>
Preamble::import () const {
  return m_import_list;
}

std::vector<std::string>
Preamble::use () const {
  return m_use_list;
}

std::vector<Node*>
Preamble::nodes () const {
  return m_node_list;
}

std::vector<NativeCodeNode*>
Preamble::java_code_nodes () const {
  return m_java_node_list;
}

void Preamble::add_node (Node *node) {
	m_node_list.push_back (node);
}
void Preamble::remove_node (Node *node) {
  m_node_list.erase (std::remove (m_node_list.begin (), m_node_list.end (), node), m_node_list.end ());
}
void Preamble::add_java_code_node (NativeCodeNode *node) {
  m_java_node_list.push_back (node);
}

void Preamble::clear () {
	m_import_list.clear ();
	m_use_list.clear ();
	m_node_list.clear ();
}
