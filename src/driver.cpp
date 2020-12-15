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

#include "driver.h"

#include "node.h"
#include "new_line_node.h"


using namespace Smala;

Driver::Driver (bool debug_mode) :
    m_scanner (*this), m_parser (m_scanner, *this), m_line (0), m_pos (0), m_error (0), _debug (false), m_debug_mode (debug_mode)
{
}

Driver::~Driver ()
{
}

void
Driver::set_is_main (bool val)
{
  m_ast.set_is_main (val);
}

int
Driver::parse ()
{
  m_line = 1;
  m_pos = 0;
  m_parser.parse ();
  return m_error;
}

void
Driver::set_error ()
{
  cerr << location() << " " << std::endl;
  m_error |= 1;
}

void
Driver::set_error (const std::string& msg)
{
  m_error |= 1;
  cerr << location() << " " << msg << std::endl;
}
void
Driver::set_stream (std::istream *is, const std::string& filename)
{
  m_file = filename;
  m_scanner.switch_streams (is, 0);
  m_ast = Ast ();
  m_loc.initialize (&const_cast<std::string&>(filename));
}

void
Driver::clear ()
{
  m_line = 0;
}

void
Driver::add_define_node (Node *node)
{
  m_ast.add_define_node (node);
}

void
Driver::add_node (Node *node)
{
  node->set_error_location (location ());
  m_ast.add_node (node);
}

void
Driver::remove_node (Node *node)
{
  m_ast.remove_node (node);
}

void
Driver::add_use (const class location& loc, const std::string &val)
{
  m_ast.add_use (loc, val);
}

void
Driver::add_import (const class location& loc, PathNode *path)
{
  m_ast.add_import (loc, path);
}

void
Driver::add_native_action (const class location& loc, const std::string &action_name, const std::string &param_name, const std::string &code)
{
  m_ast.add_native_action (loc, action_name, param_name, code);
}

void
Driver::add_native_collection_action (const class location& loc, const std::string &action_name, const std::string &list_name, const std::string &param_name, const std::string &code)
{
  m_ast.add_native_collection_action (loc, action_name, list_name, param_name, code);
}

void
Driver::add_native_code (const class location& loc, const std::string &code)
{
  m_ast.add_native_code (loc, code);
}

void
Driver::add_native_expression (NativeExpressionNode* node)
{
  m_ast.add_native_expression (node);
}

void
Driver::end_preamble ()
{
  m_ast.end_preamble ();
}


void
Driver::new_line (const class location& loc)
{
  //if (m_debug_mode)
  //  m_ast.add_node(new NewLineNode(loc, m_file, m_line));
}


const Ast&
Driver::ast ()
{
  return m_ast;
}

void
Driver::increaseLocation (const char* s, int len)
{
  bool nl = false;
  for (int i = 0; s[i] != '\0'; i++)
    if (s[i] == '\n') {
      m_line ++;
      m_pos = 0;
      nl = true;
    }
  if (!nl)
    m_pos += len;
}

smala::ErrorLocation*
Driver::location () const
{
  return new smala::ErrorLocation (m_file, m_line, m_pos);
}
