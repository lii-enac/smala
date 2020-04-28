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

#include <string>
#include <vector>
#include "node.h"

namespace Smala {

class SmalaNative : public Node
{
public:
    SmalaNative ();
    SmalaNative (const std::string &fct, const std::string &src, PathNode *data);
    ~SmalaNative ();
    
    const std::string& fct () const;
    const std::string& src () const;
    PathNode* data () const;
    
    void set_src (const std::string &src) { m_src = src; }
    void set_data (PathNode* data) { m_data = data; }

private:
    std::string m_fct;
    std::string m_src;
    PathNode* m_data;
};

}
