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

#include "smala_native.h"

#include <iostream>

using namespace Smala;

SmalaNative::SmalaNative () :
    Node (), m_fct (""), m_src (""), m_data ("")
{
  set_node_type (SMALA_NATIVE);
}

SmalaNative::SmalaNative (const std::string &fct, const std::string &src, const std::string &data) :
    Node (), m_fct (fct), m_src (src), m_data (data)
{
  set_node_type (SMALA_NATIVE);
}


SmalaNative::~SmalaNative ()
{
}

const std::string&
SmalaNative::fct () const
{
  return m_fct;
}

const std::string&
SmalaNative::src () const
{
  return m_src;
}

const std::string&
SmalaNative::data () const
{
  return m_data;
}
