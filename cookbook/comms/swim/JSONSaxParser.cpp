#include <iostream>

#include "JSONSaxParser.h"

#include "core/ontology/process.h"
#include "core/control/spike.h"
#include "core/execution/graph.h"
#include "core/core-dev.h" // GRAPH_EXEC
#include "core/control/blank.h"
#include "core/tree/component.h"
#include "core/property/double_property.h"

#include "core/utils/error.h"

namespace djnn {
    JSONSaxParser::JSONSaxParser (ParentProcess * parent, const std::string& name, const std::string& xpath)
    : FatProcess (name), _absolute (false)
    {
        assert(!xpath.empty());

        std::istringstream f(xpath);
        std::string path_component;

        auto next_sep = shallow_sep;
        _xpath_vec.push_back ("__root");
        _full_xpath_vec.push_back ("");
        _seps.push_back (shallow_sep);

        if (xpath[0]=='/') {
            getline(f, path_component, '/');
            next_sep = shallow_sep;
        } else {
            next_sep = deep_sep;
        }
        
        std::string acc_path = "";

        //auto next_sep = shallow_sep;
        while (getline(f, path_component, '/')) {
            //cout << s << endl;
            if (!path_component.empty()) {
                //new UndelayedSpike (this, s);
                //new Blank (this, s);
                //new Blank (this, s);
                acc_path += path_component;
                //std::cerr << acc_path << std::endl;
                auto * c = new Component (this, acc_path);
                if (path_component == "number") {
                    new DoubleProperty(c, "value", 0);
                }
                _full_xpath_vec.push_back (acc_path);
                acc_path += "/";
                _xpath_vec.push_back (path_component);
                _seps.push_back (next_sep);
                next_sep = shallow_sep;
            } else {
                next_sep = deep_sep;
            }
        }
        //for (auto it: _xpath_vec) {std::cerr << it << " ";} std::cerr << std::endl;
        //for (auto it: _seps) {std::cerr << it << " ";} std::cerr << std::endl;
        
        finalize_construction (parent, name); // no, we are not regular processes
    }

    FatChildProcess*
    JSONSaxParser::find_child_impl (const std::string& s)
    {
        //std::cerr << "find " << s << std::endl;
        const std::string value_string("value");
        const size_t value_size = value_string.size();
        if ( (s.size() > value_size) && (s.substr(s.size()-value_size, s.size()) == value_string)) {
            const std::string t = s.substr(0, s.size()-value_size-1);
            auto it = find_child_iterator(t);
            assert(it!=children_end());
            auto * n = it->second;
            //auto * n = FatProcess::find_child_impl(t);
            return n->find_child(value_string);
        } else {
            auto it = find_child_iterator(s);
            assert(it!=children_end());
            auto * n = it->second;
            return n;
        }
    }

    void
    //JSONSaxParser::activate (const std::string& name)
    JSONSaxParser::activate (size_t pos)
    {
        const std::string& name = _full_xpath_vec[pos];
        //std::cerr << "activating " << name << " " << pos << std::endl;
        get_exclusive_access(DBG_GET);
        auto it = find_child_iterator(name);
        assert(it!=children_end());
        auto * current = it->second;
        //std::cerr << "activate " << name << std::endl;
        current->activate (); // deactivate it
        GRAPH_EXEC;
        release_exclusive_access(DBG_REL);
    }
    void
    //JSONSaxParser::deactivate (const std::string& name)
    JSONSaxParser::deactivate (size_t pos)
    {
        const std::string& name = _full_xpath_vec[pos];
        //std::cerr << "deactivating " << name << " " << pos << std::endl;
        get_exclusive_access(DBG_GET);
        auto it = find_child_iterator(name);
        assert(it!=children_end());
        auto * current = it->second;
        //std::cerr << "deactivate " << name << std::endl;
        current->deactivate (); // deactivate it
        GRAPH_EXEC;
        release_exclusive_access(DBG_REL);
    }
    void
    //JSONSaxParser::activate_double (const std::string& name, double val)
    JSONSaxParser::activate_double (size_t pos, double val)
    {
        const std::string& name = _full_xpath_vec[pos];
        //std::cerr << "activating " << name << " " << pos << std::endl;
        get_exclusive_access(DBG_GET);
        auto it = find_child_iterator(name);
        assert(it!=children_end());
        auto * current = it->second;
        auto * dit = current->find_child("value");
        assert(dit);
        DoubleProperty * d = dynamic_cast<DoubleProperty*>(dit);
        assert(d);
        d->set_value(val, true);
        //std::cerr << "activate " << name << std::endl;
        current->activate (); // deactivate it
        GRAPH_EXEC;
        release_exclusive_access(DBG_REL);
    }
    void
    JSONSaxParser::debug_json_stack () {
        //std::cerr << "json_stack: "; for (auto s: _json_stack) { std::cerr << s << " "; } std::cerr << std::endl;
        std::cerr << "parse_stack: "; for (auto s: _parse_stack) { std::cerr << "(" << s.json << "," << s.xpath << "," << s.pos_xpath << "," << s.matching << ") "; } std::cerr << std::endl;
    }
    void
    JSONSaxParser::debug_expecting () {
        std::cerr << "expecting: " << _xpath_vec[_parse_stack.back().pos_xpath] << " " << _parse_stack.back().pos_xpath << std::endl;
    }

    void
    JSONSaxParser::parse (const std::string& input) {
        _parse_stack.clear ();
        _parse_stack.push_back (parse_stack_value_t{.json= "__root", .pos_xpath=1, .xpath= "", .matching=true});
        nlohmann::json::sax_parse(input, this);
        assert (_parse_stack.back().json == "__root");
    }

    bool
    JSONSaxParser::is_matching ()
    {
        return _parse_stack.back().json == _xpath_vec[_parse_stack.back().pos_xpath];
    }

    bool
    JSONSaxParser::start_object(std::size_t elements) {
        // std::cerr << "-- start_object" << std::endl;
        // debug_json_stack();
        // debug_expecting();

        size_t pos = _parse_stack.back().pos_xpath;
        _parse_stack.push_back (parse_stack_value_t{.json="(none)", .pos_xpath=pos, .xpath="", .matching=_parse_stack.back().matching});
        return true;
    }

    bool
    JSONSaxParser::key(json::string_t& val) {
        // std::cerr << "-- key received" << std::endl;
        // debug_json_stack();
        // debug_expecting();
        // std::cerr << "key: " << val << std::endl;
        // std::cerr << _parse_stack.back().json << " " << _xpath_vec[_parse_stack.back().pos_xpath] << " " << _parse_stack.back().pos_xpath << std::endl;
        // std::cerr << is_matching () << std::endl;


        //if (is_matching() && _parse_stack[_parse_stack.size()-2].matching) {
        //if (_parse_stack[_parse_stack.size()-2].matching) {
        if (_parse_stack.back().json==_xpath_vec[_parse_stack.back().pos_xpath-1] && _parse_stack[_parse_stack.size()-2].matching) {
            deactivate(_parse_stack.back().pos_xpath-1);
            _parse_stack.back().pos_xpath-=1;
        }

        _parse_stack.back().json = val;

        if (is_matching() && _parse_stack[_parse_stack.size()-2].matching) {
            //std::cerr << " match" << std::endl;
            auto pos = _parse_stack.back().pos_xpath;
            _parse_stack.back().xpath = val;
            _parse_stack.back().matching = true;
            _parse_stack.back().pos_xpath += 1;
            activate(pos);
        } else {
            if (_seps[_parse_stack.back().pos_xpath]==shallow_sep) {
                //std::cerr << " unmatch" << std::endl;
                //auto pos = _parse_stack.back().pos_xpath;
                _parse_stack.back().xpath = "";
                _parse_stack.back().matching = false;
            } else {
                //std::cerr << " waiting match" << std::endl;
                //auto pos = _parse_stack.back().pos_xpath;
                _parse_stack.back().xpath = "";
                _parse_stack.back().matching = true;
            }
        }
        
        return true;
    }

    bool
    JSONSaxParser::end_object() {
        // std::cerr << "-- end_object" << std::endl;
        // debug_json_stack();
        // debug_expecting();
        // std::cerr << _parse_stack.back().json << " " << _xpath_vec[_parse_stack.back().pos_xpath] << " " << _parse_stack.back().pos_xpath << std::endl;
        // std::cerr << is_matching () << std::endl;

        //if (is_matching() && _parse_stack[_parse_stack.size()-2].matching) {
        //if (_parse_stack[_parse_stack.size()-2].matching) {
        if (_parse_stack.back().json==_xpath_vec[_parse_stack.back().pos_xpath-1] && _parse_stack[_parse_stack.size()-2].matching) {
            deactivate(_parse_stack.back().pos_xpath-1);
        }

        _parse_stack.pop_back();

        
        return true;
    }

    bool
    JSONSaxParser::start_array(std::size_t elements) {
        // std::cerr << "-- start_array" << std::endl;
        // debug_json_stack();
        // debug_expecting();
        
        auto pos = _parse_stack.back().pos_xpath;
        _parse_stack.push_back (parse_stack_value_t{.json="array", .pos_xpath=pos, .xpath="", .matching=_parse_stack.back().matching});

        //_parse_stack.back().json = "array";
        //size_t pos = _parse_stack.back().pos_xpath;
        
        if (is_matching() && _parse_stack[_parse_stack.size()-2].matching) {
            activate(pos);
            if (pos<_xpath_vec.size()+1) {
                //std::cerr << "+1" <<  std::endl;
                pos += 1;
            }
            _parse_stack.back().pos_xpath = pos;
            _parse_stack.back().xpath = "array";
            _parse_stack.back().matching = true;
        } else {
            if (_seps[_parse_stack.back().pos_xpath]==shallow_sep) {
                //std::cerr << " unmatch" << std::endl;
                //auto pos = _parse_stack.back().pos_xpath;
                _parse_stack.back().xpath = "";
                _parse_stack.back().matching = false;
            } else {
                //std::cerr << " waiting match" << std::endl;
                //auto pos = _parse_stack.back().pos_xpath;
                _parse_stack.back().xpath = "";
                _parse_stack.back().matching = true;
            }
        }
        //_parse_stack.push_back (parse_stack_value_t{.json="array", .pos_xpath=pos, .xpath="array", .matching=_parse_stack.back().matching});
        
        return true;
    }
    bool
    JSONSaxParser::end_array() {
        // std::cerr << "-- end_array" << std::endl;
        // debug_json_stack();
        // debug_expecting();

        _parse_stack.pop_back();

        if (is_matching() && _parse_stack[_parse_stack.size()-2].matching) {
            deactivate(_parse_stack.back().pos_xpath);
            //_parse_stack.back().pos_xpath-=1;

        }

        
        // auto json = _parse_stack.back().json;
        //_parse_stack.back().json = "array";

        // if (is_matching() && _parse_stack[_parse_stack.size()-2].matching) {
        //     deactivate(_parse_stack.back().pos_xpath);
        //     //_parse_stack.back().pos_xpath-=1;
        // }

        // _parse_stack.back().json = json;
        
        return true;
    }
    bool
    JSONSaxParser::number_float(json::number_float_t val, const json::string_t& s) {
        // std::cerr << "-- float received: " << val << std::endl;
        // debug_json_stack();
        // debug_expecting();

        _parse_stack.back().json = "number";

        if (is_matching() && _parse_stack[_parse_stack.size()-2].matching) {
            //DBG;
            _parse_stack.back().xpath = "number";
            _parse_stack.back().matching = true;
            deactivate (_parse_stack.back().pos_xpath);
            activate_double(_parse_stack.back().pos_xpath, val);
        }
        return true;
    }
    bool
    JSONSaxParser::string(json::string_t& val) {
        // std::cerr << "-- string received: " << val << std::endl;
        // debug_json_stack();
        // debug_expecting();

        _parse_stack.back().json = "string";

        if (is_matching() && _parse_stack[_parse_stack.size()-2].matching) {
            //DBG;
            _parse_stack.back().xpath = "string";
            _parse_stack.back().matching = true;
            //deactivate (_parse_stack.back().pos_xpath);
            //activate_double(_parse_stack.back().pos_xpath, val);
        }
        return true;
    }
}

/*class UndelayedSpike : public FatProcess
{
    public:
    UndelayedSpike (ParentProcess * parent, const std::string& name)  : FatProcess (name) { set_is_model (true); finalize_construction (parent, name); }
    virtual ~UndelayedSpike () {}
    void post_activate () override { post_activate_auto_deactivate (); }
    void impl_activate () override {
        std::cerr << this << std::endl;
        for (auto& c: get_activation_couplings ()) {
            std::cerr << c << std::endl;
            c->get_dst()->notify_activation ();
            //switch(c->get_dst_activation_flag ()) {
            //    case NONE_ACTIVATION: break;
            //    case      ACTIVATION: c->get_dst()->activate ();   break;
            //    case    DEACTIVATION: c->get_dst()->deactivate (); break;
            //}
        }
    }
    void impl_deactivate () override {};
};*/

