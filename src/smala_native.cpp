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

SmalaNative::SmalaNative (const location& loc) :
    Node (loc, SMALA_NATIVE), m_fct (""), m_src (""), m_data (nullptr)
{
}

SmalaNative::SmalaNative (const location& loc, const std::string &fct, const std::string &src, PathNode* data) :
    Node (loc, SMALA_NATIVE), m_fct (fct), m_src (src), m_data (data)
{
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

PathNode*
SmalaNative::data () const
{
  return m_data;
}
