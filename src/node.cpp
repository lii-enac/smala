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

#include "node.h"

using namespace Smala;


Node::Node (const location& loc, NodeType t, const std::string &value, const std::string &name,
            const std::vector< named_parameter_t > &arguments) :
    m_ignore_parent (false),m_djnn_type (value), m_name (name), m_build_name (""), m_args_spec (arguments), m_node_type (
        t), m_in_expression (false), m_location (nullptr), m_parent (nullptr), m_path (nullptr), m_has_path (false), m_duplicate_warning (true),
    m_keep_name (false), m_loc(loc), m_is_define_or_main (false), m_include_native (false)
{
}

Node::Node (const location& loc, NodeType t)
: Node (loc, t, "", "", {})
{
}

Node::Node (const location& loc, NodeType t, const std::string &value, const std::string &name)
: Node (loc, t, value, name, {})
{
}

Node::Node (const location& loc, NodeType t, const std::string &value, PathNode* path)
: Node (loc, t, "", "", {})
{
  m_path = path;
}

Node::~Node ()
{
}

void
Node::set_parent (Node *p)
{
  m_parent = p;
}

Node*
Node::parent ()
{
  return m_parent;
}

void
Node::set_error_location (smala::ErrorLocation *loc)
{
  m_location = loc;
}

smala::ErrorLocation*
Node::error_location ()
{
  return m_location;
}

std::string
Node::djnn_type () const
{
  return m_djnn_type;
}

void
Node::set_djnn_type (const std::string& type)
{
  m_djnn_type = type;
}

const std::string&
Node::name () const
{
  return m_name;
}

void
Node::set_name (const std::string &name)
{
  m_name = name;
}

std::string
Node::build_name () const
{
  return m_build_name;
}

void
Node::set_build_name(const std::string &build_name)
{
  m_build_name = build_name;
}

std::vector< named_parameter_t >
Node::args () const
{
  return m_args_spec;
}

void
Node::add_args (std::vector< named_parameter_t > &args)
{
  m_args_spec.insert (m_args_spec.end (), args.begin (), args.end ());
}

bool
Node::has_arguments ()
{
  return !m_expr_args.empty ();
}

void
Node::set_duplicate_warning (bool v)
{
  m_duplicate_warning = v;
}

bool
Node::duplicate_warning ()
{
  return m_duplicate_warning;
}

void
Node::set_node_type (NodeType type)
{
  m_node_type = type;
}

NodeType
Node::node_type () const
{
  return m_node_type;
}
