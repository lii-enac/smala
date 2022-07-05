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
