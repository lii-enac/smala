#pragma once

#include "core/ontology/process.h"
#include "exec_env/global_mutex.h"

#include <sstream>

#include <nlohmann/json.hpp>
using json = nlohmann::json;

struct json_sax {
    virtual bool null() { return true; }
    virtual bool boolean(bool val) { return true; }
    virtual bool number_integer(json::number_integer_t val) { return true; }
    virtual bool number_unsigned(json::number_unsigned_t val) { return true; }
    virtual bool number_float(json::number_float_t val, const json::string_t& s) { return true; }
    virtual bool string(json::string_t& val) { return true; }
    virtual bool binary(json::binary_t& val) { return true; }
    virtual bool start_object(std::size_t elements) { return true; }
    virtual bool end_object() { return true; }
    virtual bool start_array(std::size_t elements) { return true; }
    virtual bool end_array() { return true; }
    virtual bool key(json::string_t& val) { return true; }
    virtual bool parse_error(std::size_t position, const std::string& last_token, const json::exception& ex) { return true; }
};

namespace djnn {
    class JSONSaxParser : public FatProcess, public json_sax {
    public:
        JSONSaxParser (ParentProcess * parent, const std::string& name, const std::string& xpath);
        void parse (const std::string& input);

        virtual bool start_object(std::size_t elements) override;
        virtual bool end_object() override;
        virtual bool key(json::string_t& val) override;
        virtual bool start_array(std::size_t elements) override;
        virtual bool end_array() override;
        virtual bool number_float(json::number_float_t val, const json::string_t& s) override;

        void impl_activate () override {}
        void impl_deactivate () override {}
    private:
        void activate (const std::string&);
        void deactivate (const std::string&);
        void debug_json_stack ();
        void debug_expecting ();
        bool is_matching ();
        bool _absolute;
        enum sep_t { shallow_sep, deep_sep};
        std::vector<std::string> _xpath_vec;
        std::vector<sep_t> _seps;
        size_t pos;
        std::vector<std::string> _json_stack;
        std::vector<bool> _matching;
        //std::vector<CoreProcess*> _process_stack;
    };
}
