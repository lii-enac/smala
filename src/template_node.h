/*
*  djnn Smala compiler
*
*  The copyright holders for the contents of this file are:
*    Ecole Nationale de l'Aviation Civile, France (2022)
*  See file "license.terms" for the rights and conditions
*  defined by copyright holders.
*
*
*  Contributors:
*     Mathieu Poirier <mathieu.poirier@enac.fr>
*     Stephane Conversy <stephane.conversy@enac.fr>
*
*/
#pragma once

#include "node.h"

namespace Smala {

    class TemplatePropertyNode : public Node {
    public:
        
        TemplatePropertyNode (const location& loc, NodeType type, const std::string &value, const std::string &name, const std::string& template_type_name)
        : Node (loc, type, value, name),
        _template_type_name (template_type_name)
        {}

        const std::string& get_template_type_name () const { return _template_type_name; }

    private:

        std::string _template_type_name;

    };

}
