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

#include <iostream>

using namespace Smala;

Node::Node () :
    m_djnn_type (""), m_name (""), m_build_name (""), m_args (), m_node_type (
        SIMPLE), m_location (nullptr), m_parent (nullptr), m_has_arguments (
        false), m_duplicate_warning (true) {
}

Node::Node (const std::string &type, const std::string &name,
            const std::vector<std::pair<ParamType, std::string> > &arguments) :
    m_djnn_type (type), m_name (name), m_build_name (""), m_args (arguments), m_node_type (
        SIMPLE), m_location (nullptr), m_parent (nullptr), m_has_arguments (
        false), m_duplicate_warning (true) {
}

Node::Node (const std::string &type, const std::string &name) :
    m_djnn_type (type), m_name (name), m_build_name (""), m_args (), m_node_type (
        SIMPLE), m_location (nullptr), m_parent (nullptr), m_has_arguments (
        false), m_duplicate_warning (true) {
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
Node::set_location (smala::ErrorLocation *loc)
{
  m_location = loc;
}

smala::ErrorLocation*
Node::location ()
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

std::vector< std::pair<ParamType, std::string> >
Node::args () const
{
  return m_args;
}

void
Node::add_args (std::vector< std::pair<ParamType, std::string> > &args)
{
  m_args.insert (m_args.end (), args.begin (), args.end ());
}

void
Node::set_has_arguments (bool v)
{
  m_has_arguments = v;
}

bool
Node::has_arguments ()
{
  return m_has_arguments;
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
