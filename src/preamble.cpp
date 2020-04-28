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
#include <algorithm>


using namespace Smala;

Preamble::Preamble () :
  m_node_list ()
{
}

Preamble::~Preamble()
{
}

void
Preamble::add_import (PathNode *path) {
  std::string ns, name;
  int sz = path->get_subpath_list ().size ();
  if (path->get_subpath_list ().size () == 1) {
    ns = path->get_subpath_list ().at (0)->get_subpath();
  }
  else {
    //ns = import.substr (0, pos);
    name = path->get_subpath_list ().at (sz - 1)->get_subpath();
  }
  Node *n = new Node (IMPORT, "", path);
  add_node (n);
  if (!name.empty ())
    m_import_list.push_back (name);
  else
    m_import_list.push_back (ns);
}

void
Preamble::add_use (const std::string &use)
{
  Node *n = new Node (USE, "", use);
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

void Preamble::add_node (Node *node) {
	m_node_list.push_back (node);
}
void Preamble::remove_node (Node *node) {
  m_node_list.erase (std::remove (m_node_list.begin (), m_node_list.end (), node), m_node_list.end ());
}

void Preamble::clear () {
	m_import_list.clear ();
	m_use_list.clear ();
	m_node_list.clear ();
}
