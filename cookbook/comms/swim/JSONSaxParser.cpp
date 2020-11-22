#include <iostream>

#include "JSONSaxParser.h"

#include "core/ontology/process.h"
#include "core/tree/spike.h"
#include "core/execution/graph.h"
#include "core/tree/blank.h"
#include "core/tree/component.h"

namespace djnn {
    JSONSaxParser::JSONSaxParser (ParentProcess * parent, const std::string& name, const std::string& xpath)
    : FatProcess (name), _absolute (false)
    {
        assert(!xpath.empty());

        std::istringstream f(xpath);
        std::string s;

        if (xpath[0]=='/') {
            getline(f, s, '/');
            _xpath_vec.push_back ("__root");
            _absolute = true;
            _seps.push_back (shallow_sep);
        } else {
            //_seps.push_back (deep_sep);
            _xpath_vec.push_back ("__root"); // temporary
        }
        
        auto next_sep = shallow_sep;
        while (getline(f, s, '/')) {
            //cout << s << endl;
            if (!s.empty()) {
                //new UndelayedSpike (this, s);
                //new Blank (this, s);
                //new Blank (this, s);
                new Component (this, s);
                _xpath_vec.push_back (s);
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

    void
    JSONSaxParser::activate (const std::string& name)
    {
        //std::cerr << "activating " << name << std::endl;
        get_exclusive_access(DBG_GET);
        auto it = find_child_iterator(name);
        assert(it!=children_end());
        auto * current = it->second;
        current->activate (); // deactivate it
        GRAPH_EXEC;
        release_exclusive_access(DBG_REL);
    }
    void
    JSONSaxParser::deactivate (const std::string& name)
    {
        //std::cerr << "deactivating " << name << std::endl;
        get_exclusive_access(DBG_GET);
        auto it = find_child_iterator(name);
        assert(it!=children_end());
        auto * current = it->second;
        current->deactivate (); // deactivate it
        GRAPH_EXEC;
        release_exclusive_access(DBG_REL);
    }
    void
    JSONSaxParser::debug_json_stack () {
        std::cerr << "json_stack: "; for (auto s: _json_stack) { std::cerr << s << " "; } std::cerr << std::endl;
    }
    void
    JSONSaxParser::debug_expecting () {
        std::cerr << "expecting: " << _xpath_vec[pos] << std::endl;
    }

    void
    JSONSaxParser::parse (const std::string& input) {
        pos = 0;
        using json = nlohmann::json;
        _json_stack.clear (); // should not be necessary
        _json_stack.push_back ("__root");
        _matching.clear ();
        _matching.push_back (true);
        json::sax_parse(input, this);
        assert (_json_stack.back() == "__root");
    }

    bool
    JSONSaxParser::is_matching ()
    {
        return _json_stack.back() == _xpath_vec[pos];
        /*return _seps[pos]==shallow_sep
            ? _json_stack.back() == _xpath_vec[pos]
            : _matching.back();*/
    }

    bool
    JSONSaxParser::start_object(std::size_t elements) {
        // std::cerr << "-- start_object" << std::endl;
        // debug_json_stack();
        // debug_expecting();

        if (is_matching()) {
            if (pos<_xpath_vec.size()) {
                pos += 1;
            }
        }    
        _json_stack.push_back("(none)"); // we don't know yet the name
        _matching.push_back(false);
        return true;
    }

    bool
    JSONSaxParser::key(json::string_t& val) {
        // std::cerr << "-- key received: " << val << std::endl;
        // debug_json_stack();
        // debug_expecting();

        if (is_matching()) {
            //_matching.back()=false;
            _matching.pop_back();
            _matching.push_back(_matching.back());
            deactivate(_json_stack.back());
        }

        _json_stack.back() = val;

        if (is_matching()) {
            _matching.back()=true;
            activate(_json_stack.back());
        }
        
        return true;
    }

    bool
    JSONSaxParser::end_object() {
        // std::cerr << "-- end_object" << std::endl;
        // debug_json_stack();
        // debug_expecting();

        if (is_matching()) {
            deactivate(_json_stack.back());
            if (pos>0) {
               pos -= 1;
            }
        }

        _json_stack.pop_back();
        _matching.pop_back();
        return true;
    }

    bool
    JSONSaxParser::start_array(std::size_t elements) {
        /*if(!_json_stack.empty() ) {
            if (_json_stack.back() == _xpath_vec[pos]) { // if we were handling a segment of xpath...
                if (pos<_xpath_vec.size() && (_xpath_vec[pos+1]=="list")) {
                    get_exclusive_access(DBG_GET);
                    auto it = find_child_iterator("list");
                    assert(it!=children_end());
                    auto * current = it->second;
                    current->activate (); // activate it
                    GRAPH_EXEC;
                    release_exclusive_access(DBG_REL);
                    pos += 1;
                }
            }    
        }
        _json_stack.push_back("list");*/
        return true;
    }
    bool
    JSONSaxParser::end_array() {
        /*if(!_json_stack.empty() ) {
            if (_json_stack.back() == _xpath_vec[pos]) { // if we were handling a segment of xpath...
                get_exclusive_access(DBG_GET);
                auto it = find_child_iterator("list");
                assert(it!=children_end());
                auto * current = it->second;
                current->deactivate (); // deactivate it
                GRAPH_EXEC;
                release_exclusive_access(DBG_REL);
                if (pos>0) {
                    pos -= 1;
                }
            }
        }
        _json_stack.pop_back();*/
        return true;
    }
    bool
    JSONSaxParser::number_float(json::number_float_t val, const json::string_t& s) { return true; }
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

