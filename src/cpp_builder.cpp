
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
 *      Stephane Conversy <stephane.conversy@enac.fr>
 *
 */

#include "cpp_builder.h"
#include "native_expression_node.h"
#include "instruction_node.h"
#include "binary_instruction_node.h"
#include "smala_native.h"
#include "ctrl_node.h"
#include "cpp_type_manager.h"
#include "for_node.h"
#include "transition_node.h"
#include "native_collection_action_node.h"
#include "forevery_node.h"
#include "causal_dep_node.h"
#include "self_assign_node.h"
#include "template_node.h"

#include <locale>
#include <algorithm>
#include <unistd.h> // getpid
#include "core/utils/filesystem.h"

#include <sstream>
#include <fstream>
#include <experimental/iterator>

//#define BOOST_STACKTRACE_USE_ADDR2LINE
//#include <boost/stacktrace.hpp>


//#include <stdio.h>
/*
#include <execinfo.h>
string print_trace(void) {
    char **strings;
    size_t i, size;
    enum Constexpr { MAX_SIZE = 1024 };
    void *array[MAX_SIZE];
    size = backtrace(array, MAX_SIZE);
    strings = backtrace_symbols(array, size);
    string res;
    for (i = 0; i < size; i++) {
        //printf("%s\n", strings[i]);
        res += string(strings[i]) + " ";
    }
    //puts("");
    free(strings);
    return res;
}
*/

const bool m_fastcomp = false; // global for now, make it an option later

//#define emit_compiler_info(OS)
#define emit_compiler_info(OS) { indent(OS); OS << "// code emited by " << __PRETTY_FUNCTION__ << ":" << __FILE__ << ":" << __LINE__ << "\n"; }

namespace Smala
{
  static std::string
  transform_name(const std::string& input)
  {
    std::string new_param_name = input;
    std::replace(new_param_name.begin(), new_param_name.end(), '.','_');
    std::replace(new_param_name.begin(), new_param_name.end(), '/','_');
    std::replace(new_param_name.begin(), new_param_name.end(), '-','_');
    std::replace(new_param_name.begin(), new_param_name.end(), '>','_');
    new_param_name.erase(std::remove(new_param_name.begin(), new_param_name.end(), '"'), new_param_name.end());
    return new_param_name;
  }

  CPPBuilder::CPPBuilder () :
      Builder (), m_display_initialized (false), m_expr_in (0), m_expr_out (0)
  {
    m_type_manager = new CPPTypeManager ();
    m_null_symbol = "nullptr";
    m_null_string = "\"\"";
  }

  CPPBuilder::~CPPBuilder ()
  {
  }

  void
  CPPBuilder::push_ctxt (const string& parent_name)
  {
    //std::cerr << ">> " << sym_stack.size() << " push " << parent_name << std::endl;
    Builder::push_ctxt (parent_name);
    sym_stack.push_back (sym);
    prop_sym_stack.push_back (prop_sym);
    m_template_props_stack.push_back(m_template_props);
  }

  void
  CPPBuilder::pop_ctxt ()
  {
    //std::cerr << "<< " << sym_stack.size() << " pop" << std::endl;
    Builder::pop_ctxt ();

    sym_stack.pop_back ();
    //if(!sym_stack.empty()) // FIXME should never be empty...
      sym = sym_stack.back ();

    prop_sym_stack.pop_back ();
    //if(!prop_sym_stack.empty())
      prop_sym = prop_sym_stack.back ();
    
    m_template_props_stack.pop_back ();
    //if(!m_template_props_stack.empty ())
      m_template_props = m_template_props_stack.back ();
  }

  

  int
  CPPBuilder::build (const Ast &ast, const std::string &builddir,
                     const std::string &prefix, bool debug, bool cleaner)
  {
    m_cleaner = cleaner;
    m_indent = 0;
    m_cpnt_num = 0;
    m_var_num = 0;
    m_ast = ast;
    m_types.clear ();
    m_parent_list.clear ();

    m_parent_list.push_back (new BuildNode (m_null_symbol)); // the first parent is null
    sym_stack.push_back (sym);
    prop_sym_stack.push_back (prop_sym);
    m_template_props_stack.push_back(m_template_props);

    std::string sep="/";
    // if (!builddir.empty()) {
    //   if (builddir[builddir.size()-1] != '/')
    //     sep = "/";
    // }

    if (!ast.is_main ()) {
      std::string header_path;
      if (!builddir.empty())
        header_path = builddir + sep + prefix;
      else
        header_path = prefix;
      build_header (header_path); // FIXME should open a temporary file for header .h
    }

    // create a temporary output file that will be copied into the final output file
    // when we know the includes we need
    //auto tmp_file_name = prefix + std::to_string(getpid()) + ".cpp";
    auto tmp_file_name = std::filesystem::temp_directory_path().string() + sep + "smalac_tmp_" + std::to_string(getpid()) + ".cpp";
    std::ofstream os (tmp_file_name);
    if (!os.good()) {
      std::cerr << "temporary file " << tmp_file_name << " not good" << std::endl;
    }

    // change m_filename, which will be reused in native expression name
    if (!builddir.empty())
        m_filename = builddir + sep + prefix + ".cpp";
    else
        m_filename = prefix + ".cpp";
    //std::ostream os (prefix + ".cpp");

    emit_compiler_info(os);

    if (!m_fastcomp) {
      if(m_ast.is_main ()) {
          build_use(os, "exec_env");
      }

      //if (debug) {
        os << "#include \"core/utils/error.h\" // for Context class\n";
        os << "#undef error // avoid name clash with error macro and possible following #include\n";
        os << "#undef warning // avoid name clash with error macro and possible following #include\n\n";
      //}
    } else {
      os << R"(
int djnn__error (const djnn::CoreProcess *p, const char* msg, const char* ctxinfo=nullptr);
void djnn__warning (const djnn::CoreProcess *p, const char* msg, const char* ctxinfo=nullptr);
int djnn__exit(int ret);// { exit(ret); return 1; }
)";
    }
    //os << "namespace djnn { extern void init_exec_env (); extern void clear_exec_env();\n"; // no need to use exec_env in smala program, or should it in main? 

    //os << "using namespace std;\n";
    os << "using namespace djnn;\n\n";

    indent (os);
    os << "extern \"C\" { int puts(const char *s); } // for print()\n";

    int size = m_ast.preamble ().import ().size ();
    for (int i = 0; i < size; ++i) {
      /* add the import name to the possible types */
      std::string name = m_ast.preamble ().import ().at (i);
      m_types.insert (std::pair<std::string, std::string> (name, name));
      m_import_types.insert (std::pair<std::string, std::string> (name, name));
    }
    if(debug)
      os << "\n#line 1" << " \"" << m_filename << "\"" << std::endl;
    build_preamble (os, debug);

    size = m_ast.node_list ().size ();
    location last_loc;

    for (int i = 0; i < size; ++i) {
      Node * node = m_ast.node_list ().at (i);
      
      if(debug) {
        const location & loc = node->get_location();
        if(loc.begin.line != last_loc.begin.line) {
          auto * f = node->get_location().begin.filename;
          os << "\n#line " << node->get_location().begin.line << std::endl; //" \"" <<  (f?*f:std::string("")) << "\"" << std::endl;
          if (node->node_type () == START_ELSEIF || node->node_type () == START_ELSE)
            m_in_code = false;
          if (m_in_code) {
              os << "Context::instance()->parser_info(" 
                 << node->get_location().begin.line << ", "
                 << node->get_location().begin.column << ", "
                 << node->get_location().end.line << ", "
                 << node->get_location().end.column << ", "
                 << "\"" << (f?*f:std::string("")) << "\""
                 << ");"
                 << std::endl;
          }
          if (node->node_type () == START_IF || node->node_type () == END_BLOCK) {
            m_in_code = true;
          }
          last_loc = loc;
        }
      }

      build_node (os, node);

      if (node->is_define_or_main()) {
        m_define_or_main_node = node;
      }

      if(debug) {
        if (node->node_type()==START_MAIN || node->node_type()==START_DEFINE)
          m_in_code = true;
        if (node->node_type()==END_MAIN || node->node_type()==END_DEFINE)
          m_in_code = false;
      }

    }
    if (m_ast.is_main ())
      os << "}\n";
    os.close ();

    // if(!builddir.empty()) {
    //   m_filename = builddir + sep + prefix + ".cpp";
    // } else {
    //   m_filename = prefix + ".cpp";
    // }

    std::ofstream final_os (m_filename);
    if (!final_os.good()) {
      std::cerr << "dest file " << m_filename << " not good" << std::endl;
    }

    auto sav_ident = m_indent;
    m_indent = 0;
    emit_compiler_info(final_os);
    m_indent = sav_ident;

    if (m_fastcomp) { 
      final_os << "#include \"c_api/djnn_c_api.h\"\n";
    } else {
      final_os << "#include \"core/utils/containers/string.h\"" << std::endl;
      final_os << "using djnnstl::string;" << std::endl;
      final_os << "using djnnstl::to_string;" << std::endl;
      for (auto p: used_processes) {
        extern std::map<std::string, std::string> process_class_path;
        try {
          auto include = process_class_path[p.first];
          if (include != "") {
            //std::cerr << p.first.c_str() << std::endl;
            final_os << "#include \"" << include << "\"\n";
          }
        }
        catch(std::out_of_range&) {
        }
      }
    }
    for (auto p: used_headers) {
      final_os << "#include \"" << p.first << "\"\n";
    }
    final_os << "\n";

    std::ifstream os2 (tmp_file_name);
    if (!os2.good()) {
      std::cerr << "temp file reading not good" << std::endl;
    }
    final_os << os2.rdbuf ();
    final_os.close();
    os2.close();

    filesystem::remove(tmp_file_name);

    return m_error;
  }

  void
  CPPBuilder::build_use (std::ostream &os, std::string use)
  {
    emit_compiler_info(os);
    /*if (use.compare ("core") == 0)
      return;
    else*/
    if (!m_fastcomp) {
      if (use.compare ("gui") == 0) {
        if (!m_display_initialized) {
          os << "#include \"display/display-dev.h\"\n";
          //os << "namespace djnn { extern void init_display (); extern void clear_display(); }\n"; 
          m_display_initialized = true;
        }
      }
      else
      if (use.compare ("display") == 0) {
        if (m_display_initialized)
          return;
        m_display_initialized = true;
      }
      os << "#include \"" << use << "/" << use << "-dev.h\"\n";
    }
    
    //os << "namespace djnn { extern void init_" << use << "(); extern void clear_" << use << "(); }\n"; 
  }

  void
  CPPBuilder::build_import (std::ostream &os, Node *n)
  {
    emit_compiler_info(os);
    os << "#include \"";
    auto sep = "/";
    auto *sep_2 ="";
    for (auto sub : n->get_path()->get_subpath_list()) {
      os << sep_2 << sub->get_subpath();
      sep_2 = sep;
    }
    os << ".h\"\n";
  }

  void
  CPPBuilder::build_post_import (std::ostream &os)
  {
    emit_compiler_info(os);
    os << "inline\n";
    os << "double smala_deref(djnn::AbstractProperty& p)\n";
    os << "{ return p.get_double_value(); }\n\n";

    os << "inline\n";
    os << "const djnnstl::string& smala_deref(const djnnstl::string& p)\n";
    os << "{ return p; }\n\n";

    os << "inline\n";
    os << "double smala_deref(double p)\n";
    os << "{ return p; }\n\n";

    used_processes["AbstractProperty"] = true;
  }

  void
  CPPBuilder::build_start_if (std::ostream &os, Node* n)
  {
    emit_compiler_info(os);
    std::string expr_str = build_expr (n->get_args().at(0));
    build_properties(os);

    if (!m_after_else) {
      indent (os);
    }

    //indent (os);
    os << "if ("
       << expr_str
       << ") {\n";

    m_indent++;
  }

  void
  CPPBuilder::build_start_else (std::ostream &os)
  { 
    os << "else {\n";
  }
  
  void
  CPPBuilder::build_start_else_if (std::ostream &os)
  {
    //pop_ctxt();
    os << "else {\n";
    push_ctxt(); //DBG;
  }


  // since build_expr is likely to make use of new AbstractProperties,
  // we need to actually create them
  // the idiom is like this:
  //           std::string expr_str = build_expr (n->get_args().at(0));
  //           build_properties(os);
  // in 'build_expr', each time there is a need for an AbstractProperty, we push it into m_new_syms_from_build_expr
  // when it's safe to do so (not inside an if () expr for example), we call build_properties()

  void
  CPPBuilder::build_properties(std::ostream &os)
  {
    if ( m_new_syms_from_build_expr.empty() ) return;
    emit_compiler_info(os);
    for (auto const & e: m_new_syms_from_build_expr) {
      indent (os);
      os << "[[maybe_unused]] auto * " << e.second << " = ";
      if (!m_fastcomp) {
        os << "dynamic_cast<AbstractProperty*> (" << e.first << ");" << endl;
      } else {
        os << "(" << e.first << ");" << endl;
      }
    }
    m_new_syms_from_build_expr.clear();
  }


  // separate between a 'external' call and a recursive, 'internal' call
  // this used to be useful when debugging, so stick to this scheme 
  std::string
  CPPBuilder::build_expr (ExprNode *e, expr_production_t prod_t, bool build_fake)
  {
    std::string res;
    res = build_expr_rec (e, prod_t, build_fake);
    return res;
  }

  std::string
  CPPBuilder::build_expr_rec (ExprNode *e, expr_production_t prod_t, bool build_fake)
  {
    std::string expr;
    if (e->is_enclosed_with_parenthesis())
      expr += "(";
    switch (e->get_expr_node_type()) {
      case LITERAL: {
        if (e->get_expr_type() == NULL_VALUE)
          expr += "nullptr";
        else
          expr += e->get_val();
        break;
      }
      case PATH_EXPR: {
        std::string path;
        if (build_fake) {
          path = transform_name (build_fake_name(((PathExprNode*)e)->get_path(), false));
        } else {
          auto & whatever_name = ((PathExprNode*)e)->get_path()->get_subpath_list().at (0)->get_subpath ();
          if (m_in_switch)
            path = "\"" + whatever_name + "\"";
          else {
            path += build_path (((PathExprNode*)e)->get_path());
            // if no path is found check for a global symbol
            if (path.empty())
              path += m_type_manager->get_smala_symbol (whatever_name);
            // if path is still empty, keep it 
            if (path.empty())
              path += whatever_name;
          }
        }
        if (prod_t == string_t || e->get_expr_type() == CAST_STRING) {
          if (!m_fastcomp) {
            expr += "((AbstractProperty*)" + path + ")->get_string_value ()"; // FIXME
          } else {
            expr += "djnn_get_string_value (" + path + ")"; // FIXME
          }
          used_processes["AbstractProperty"] = true;
        } else if (m_in_switch || prod_t == process_t || e->get_expr_type() != PROCESS) {
          expr += path;
          m_in_switch = false;
        } else {
          std::string new_path = path; //transform_name (path);
          if (!build_fake) { // only used in native_expr action, use it as "inside native expr")
            if (prop_sym.find(new_path) != prop_sym.end()) {
              new_path = prop_sym[path];
            }
            else {
              auto new_sym = std::string ("cpnt_" + std::to_string (m_cpnt_num++)); //"TODO"; //transform_name (path);
              sym[new_path] = new_sym;
              m_new_syms_from_build_expr[new_path] = new_sym;
              prop_sym[new_path] = new_sym;
              //std::cerr << "**" << new_path << " " << new_sym << std::endl;
              new_path = new_sym;
            }
          }

          if (!m_fastcomp) {
            expr += "get_property_value (" + new_path + ")"; // + print_trace(); // + boost::stacktrace::stacktrace();
          } else {
            expr += string("djnn_") + "get_double_value (" + new_path + ")"; // + print_trace(); // + boost::stacktrace::stacktrace();
          }
        }
        break;
      }
      case STEP: {
        expr += build_step (e);
        break;
      }
      case FUNCTION: {
        expr += e->get_val () + " (";
        used_processes[e->get_val ()] = true;
        std::string sep = "";
        for (auto sub : ((FunctionExprNode*)e)->get_args()) {
          expr += sep + build_expr_rec (sub, prod_t, build_fake);
          sep = ",";
        }
        expr += ")";
        break;
      }
      case UNARY_OP: {
        UnaryExprNode *un = (UnaryExprNode*) e;
        if (un->get_val().compare ("$") == 0 && (un->get_child()->get_expr_type() == PROCESS)) {
          expr += build_expr_rec (un->get_child(), number_t, build_fake);
        } else if (un->get_val().compare ("&") == 0 && (un->get_child()->get_expr_type() == PROCESS)) {
          expr += build_expr_rec (un->get_child(), process_t, build_fake);
        } else if (un->get_val().compare ("$") == 0 || un->get_val().compare ("&") == 0){
          expr += build_expr_rec (un->get_child(), prod_t, build_fake);
        } else {
          expr += un->get_val () + " " + build_expr_rec (un->get_child(), prod_t, build_fake);
        }
        break;
      }
      case BINARY_OP: {
        BinaryExprNode *bin = (BinaryExprNode*) e;
        if (bin->get_val().compare("+") == 0 || bin->get_val().compare("==") == 0) {
          if (is_string (bin->get_left_child()) && bin->get_right_child()->get_expr_type() == PROCESS) {
            expr += build_expr_rec (bin->get_left_child(), string_t, build_fake) + " " + bin->get_val() + " ";
            expr += build_expr_rec (bin->get_right_child(), string_t, build_fake);
          } else if (bin->get_left_child()->get_expr_type() == PROCESS && is_string (bin->get_right_child())) {
            expr += build_expr_rec (bin->get_left_child(), string_t, build_fake);
            expr += " " + bin->get_val () + " " + build_expr_rec (bin->get_right_child (), string_t, build_fake);
          } else {
            expr += build_expr_rec (bin->get_left_child (), prod_t, build_fake) + " " + bin->get_val () + " " + build_expr_rec (bin->get_right_child (), prod_t, build_fake);
          }
        } else {
          if (bin->get_val().compare("%") == 0)
            expr += "(int)"; // hum! hum! are we sure that we want to force the cast
          expr += build_expr_rec (bin->get_left_child (), prod_t, build_fake) + " " + bin->get_val () + " ";
          if (bin->get_val().compare("%") == 0)
            expr += "(int)"; // hum! hum! are we sure that we want to force the cast
          expr += build_expr_rec (bin->get_right_child (), prod_t, build_fake);
        }
        break;
      }
      case TERNARY_OP: {
        TernaryExprNode *ter = (TernaryExprNode*) e;
        expr += build_expr_rec (ter->get_condition(), undefined_t, build_fake) + " ? " + build_expr_rec (ter->get_left_child (), prod_t, build_fake) + " : " + build_expr_rec (ter->get_right_child(), prod_t, build_fake);
        break;
      }
      case ARRAY: {
        auto *an = dynamic_cast<ArrayVarNode*> (e);
        assert (an);
        std::string type;
        switch (an->get_array_type()) {
        case DOUBLE:
          type = "double";
          break;
        case INT:
          type = "int";
          break;
        case PROCESS:
          type = "Process*";
          break;
        default:
          type = "string";
        }

        expr += "vector<" + type +"> {";
        std::string pre = "";
        if (an->get_array_type() != PROCESS) {
          //for (auto v: an->get_val_array()) {
          //expr += pre + v;
          //TODO
          //}
        } else {
          //for (auto v: an->get_process_array()) {
            //expr += pre + build_path(v);
        //pre = ", ";
          //}
        }
      expr += "}";
      }
      break;
    }
    if (e->is_enclosed_with_parenthesis())
      expr += ")";
    return expr;
  }

  void
  CPPBuilder::build_component (std::ostream &os, const std::string &var_name, const std::string &constructor, std::string &parent_name, std::string &child_name, Node* node)
  {
    emit_compiler_info(os);

    // handle properties
    // FIXME? handling properties and templates should probably be somewhere else
    bool is_prop = false;
    static const vector<string> prop_components = {"IntProperty", "DoubleProperty", "TextProperty", "BoolProperty", "TemplateProperty"};
    //std::cerr << constructor << std::endl;
    if (find(begin(prop_components), end(prop_components), constructor) != end(prop_components)) {
      is_prop = true;
      if (constructor == "TemplateProperty") {
        m_template_props[var_name] = true;
        //std::cerr << " _ " << var_name << std::endl;
      }
    }

    if (is_prop) {
      auto tname = transform_name(child_name);
      prop_sym[tname] = var_name;
      PathNode * pn = node->get_path ();
      if (pn) {
        auto * tmpl_node = dynamic_cast<TemplatePropertyNode*>(node);
        assert(tmpl_node);
        indent (os);
        os << "[[maybe_unused]] auto * " << var_name << " = dynamic_cast<TemplateProperty<" << tmpl_node->get_template_type_name() << ">*> (" << parent_name << "->find_child(\"" << pn->build_string_repr("/") << "\"));\n";
        used_processes["TemplateProperty"] = true;
        return;
      } //else ?!
    }

    // handle constructor arguments
    std::string args_str;
    for (auto sub : node->get_args ()) {
      args_str += ", " + build_expr (sub);
    }
    build_properties(os);

    // emit something like "auto * var_name = constructor"
    //indent (os);
    print_start_component (os, var_name, constructor);

    // emit arguments
    os << " (" << parent_name << ", " << child_name;
    os << args_str;
    os << ");\n";
    
  }
    

  void
  CPPBuilder::print_start_component (std::ostream &os, const std::string &var_name, const std::string &constructor)
  {
    emit_compiler_info(os);
    indent (os);
    print_component_decl (os, var_name);
    os << " = ";
    print_component_constructor (os, constructor);
    used_processes[constructor] = true;
  }

  void
  CPPBuilder::build_component_arguments (std::ostream &os, std::string &parent_name, std::string &child_name, Node* node)
  {
    // subsumed by build_component


    /*
    std::string args_str;
    for (auto sub : node->get_args()) {
      args_str += ", " + build_expr(sub);
    }
    build_properties(os);

    os << " (" << parent_name << ", " << child_name;
    os << args_str;
    os << ");\n";*/
  }

  void
  CPPBuilder::build_range_node (std::ostream &os, Node *node, const string& new_name)
  {
    RangeNode* n = dynamic_cast<RangeNode*> (node);
    std::string name = node->name ().empty () ? m_null_string : "\"" + node->name () + "\"";
    std::string p_name = (node->parent () == nullptr || node->ignore_parent ()) ? m_null_symbol : node->parent ()->build_name ();

    std::string args_str;
    auto lower_str = build_expr(n->lower_arg());
    auto upper_str = build_expr(n->upper_arg());
    emit_compiler_info(os);
    build_properties(os);

    emit_compiler_info(os);
    indent (os);
    print_start_component (os, new_name, "SwitchRangeBranch");
    os << " (" << p_name << ", " << name << ", " << lower_str;
    os << ", " << n->left_open () << ", " << upper_str;
    os << ", " << n->right_open () << ");\n";
  }

  std::string
  CPPBuilder::build_fake_name (PathNode* n, bool out)
  {
    std::string fake = n->get_subpath_list ().at (0)->get_subpath();
    for (size_t i = 1; i < n->get_subpath_list ().size (); i++) {
      if (n->get_subpath_list ().at (i)->get_path_type() == EXPR) {
        if (out)
          fake += ".expr_out" + std::to_string (m_expr_out++);
        else
          fake += ".expr_in" + std::to_string (m_expr_in++);
      }
      else
        fake += "." + n->get_subpath_list ().at (i)->get_subpath ();
    }
    return fake;
  }

  std::string
  CPPBuilder::build_path (PathNode* n)
  {
    const std::vector<SubPathNode*>& n_list = n->get_subpath_list ();
    const std::string& symbol = n_list.at (0)->get_subpath ();
    //std::string str = m_parent_list.back ()->get_symbol (symbol);
    const std::string& parent_symbol = m_parent_list.back ()->get_symbol (symbol);
    

    if (n_list.size () == 1)
      return parent_symbol;
    if (n_list.size () == 2 && (n_list.at (1)->get_path_type () == WILD_CARD || n_list.at (1)->get_path_type () == PATH_LIST))
      return parent_symbol;

    std::string str = parent_symbol;
    //std::cerr << str << std::endl;

    bool in_path = false;
    for (size_t i = 1; i < n_list.size (); i++) {
      if (n_list.at (i)->get_path_type () == WILD_CARD || n_list.at (i)->get_path_type () == PATH_LIST)
        break;
      if (n_list.at (i)->get_path_type () != EXPR) {
        if (in_path)
          str += "/";
        else {
          if (!m_fastcomp) {
            str += "->find_child (\"";
          } else {
            str = string("djnn_find (") + str + ", \"";
          }
          in_path = true;
        }
        str += n->get_subpath_list ().at (i)->get_subpath ();
      }
      else {
        ExprNode* expr = n->get_subpath_list ().at (i)->get_expr ();
        if (expr->get_expr_node_type () == LITERAL && expr->get_expr_type () == INT) {
          if (in_path)
            str += "/";
          else {
            if (!m_fastcomp) {
              str += "->find_child (\"";
            } else {
              str = string("djnn_find (") + str + ", \"";
            }
            in_path = true;
          }
          str += expr->get_val ();
        } else {
          if (in_path) {
            str += "\")";
            in_path = false;
          }
          if (!m_fastcomp) {
            str += "->find_child (";
          } else {
            str = string("djnn_find (") + str + ", ";
          }
          str += build_expr (expr);
          str += ")";
        }
      }
    }
    if (in_path)
      str += "\")";
    return str;
  }

  std::string
  CPPBuilder::build_find (PathNode* n, bool ignore_cast)
  {
    const std::vector<SubPathNode*>& n_list = n->get_subpath_list ();
    if (n_list.empty ())
      return "";

    std::string str;
    std::string prefix;
    std::string symbol = n_list.at (0)->get_subpath ();
    str = m_parent_list.back ()->get_symbol (symbol);

    if (str.empty ()) {
      // then check if it is a Djnn symbol that is a key prefixed by DJN
      if (symbol.substr (0, 3) == "DJN") return symbol;
      else {
        // finally check if it is a Smala symbol
        str = m_type_manager->get_smala_symbol (symbol);
        if (!str.empty ()) return str;
      }
      // if everything fails, print an error message
      print_error_message (error_level::error, "Symbol not found: " + symbol, 1);
      return symbol;
    }
    if (str.compare (0, 6, "d_var_") == 0 || str.compare (0, 6, "i_var_") == 0
        || str.compare (0, 6, "s_var_") == 0 || str.compare (0, 4, "var_") == 0) {
      return str;
    }
    str = build_path (n);

    return prefix + str;
  }

  std::string
  CPPBuilder::build_step (ExprNode *node)
  {
    StepExprNode *step = (StepExprNode*) node;
    std::string res;
    std::string find = build_find (step->get_path(), false);
    if ( (find.find ("cpnt_", 0) == 0) || (find.find ("arg_", 0) == 0)) {
      res += "((AbstractProperty*) " + find + ")->set_value (";
      res += "((AbstractProperty*) " + find + ")->get_double_value ()";
      used_processes["AbstractProperty"] = true;
      if (step->is_incr ())
        res += " +";
      else
        res += " - ";
      res += " 1, true)";
    } else {
      res += find;
      if (step->is_incr ())
        res += "++";
      else
        res += "--";
    }
    if (!m_in_for)
      res += ";\n";
    return res;
  }

  void
  CPPBuilder::build_print (std::ostream &os, Node *node)
  {
    emit_compiler_info(os);
    ExprNode* expr = node->get_args ().at (0);
    auto expr_str = build_expr (expr, string_t);
    build_properties (os);
    
    std::string name ("var_" + std::to_string (m_var_num++));
    indent (os);
    os << "djnnstl::string " << name << " ( ";
    os << expr_str;
    os << ");\n";

    used_headers["core/utils/containers/string.h"];

    indent (os);
    //os << "djnnstl::cout << " << name << ";\n";
    os << "puts (" << name << ".c_str());\n";

  }

  void
  CPPBuilder::build_while (std::ostream &os, Node *node)
  {
    emit_compiler_info(os);
    auto expr_str = build_expr (node->get_args().at(0));
    build_properties (os);

    indent (os);
    os << "while (" << expr_str << ") {";
    os << ") {\n";
    push_ctxt (); //DBG;
    m_indent++;
  }

  void
  CPPBuilder::build_end_block (std::ostream &os)
  {
    os << "}\n";
  }

  void
  CPPBuilder::build_for (std::ostream &os, Node *node)
  {
    emit_compiler_info(os);
    ForNode *fn = (ForNode*) node;
    push_ctxt (); //DBG;
    m_in_for = true;

    std::ostringstream first_os;
    build_for_node (first_os, fn->first_st());
    build_properties(os);

    std::string expr_str = build_expr (fn->second_st());
    build_properties(os);

    std::ostringstream third_os;
    build_for_node (third_os, fn->third_st());

    build_properties(os);

    indent (os);
    os << "for (";
    os << first_os.str();
    os << "; "
       << expr_str
       << "; ";
    os << third_os.str();
    os << ") {\n";
    m_indent++;
    m_in_for = false;
  }

  void
  CPPBuilder::build_for_every (std::ostream &os, Node *node)
  {
    emit_compiler_info(os);
    ForEveryNode* n = (ForEveryNode*) node;
    std::string list_name = "list_" + std::to_string (m_cpnt_num++);
    std::string loc_name = "cpnt_" + std::to_string (m_cpnt_num++);
    indent (os);
    os << "const djnnstl::vector<CoreProcess*>* " << list_name << ";\n";
    std::string path = build_find (n->get_path (), true);
    build_properties(os);
    indent (os);
    os << "[[maybe_unused]] auto * " << loc_name << " = " << path << ";\n";
    os << "{ auto*&cpn=" << loc_name << "; auto & lst=" << list_name << ";\n";
    os << R"(
  if (dynamic_cast<ProcessCollector*> (cpn) != nullptr) {
		lst = &((ProcessCollector*) cpn)->get_list();
	} else if (dynamic_cast<Container*> (cpn) != nullptr) {
		lst = &((Container*) cpn)->children();
	} else {
		djnn__error (nullptr, "only Container and ProcessCollector can be used in forevery instruction\n");
		djnn__exit (0);
	}
  }
  )";
    /*
    indent (os);
    os << "if (dynamic_cast<ProcessCollector*> (" << loc_name << ") != nullptr) {\n";
    m_indent++;
    indent (os);
    os << list_name << " = " <<  "((ProcessCollector*) "<< loc_name << ")->get_list();\n";
    m_indent--;
    indent (os);
    os << "} else if (dynamic_cast<Container*> (" << loc_name << ") != nullptr) {\n";
    m_indent++;
    indent (os);
    os << list_name << " = " <<  "((Container*) "<< loc_name << ")->children();\n";
    m_indent--;
    indent (os);
    os << "} else {\n";
    m_indent++;
    indent (os);
    os << "djnn__error (nullptr, \"only Container and ProcessCollector can be used in forevery instruction\\n\");\n";
    indent (os);
    os << "exit (0);\n";
    m_indent--;
    indent (os);
    os << "}\n";
    */
    indent (os);
    std::string var_name = "cpnt_" + std::to_string (m_cpnt_num++);
    os << "for (auto " << var_name << " : *" << list_name << ") {\n";
    m_indent++;
  
    push_ctxt (); //DBG;
    m_parent_list.back ()->add_entry (n->get_new_name (), var_name);

    used_processes["ProcessCollector"] = true;
    used_processes["Container"] = true;
  }


  void
  CPPBuilder::build_control_node (std::ostream &os, Node *node)
  {
    emit_compiler_info(os);
    CtrlNode *ctrl = dynamic_cast<CtrlNode*> (node);
    std::string constructor = get_constructor (node->djnn_type ());

    std::string src = build_find (ctrl->in ()->get_path(), false);
    build_properties(os);
    std::string src_name, dst;
    std::string prefix = "var_";
    bool _is_var = src.substr (0, prefix.size ()) == prefix;
    if (_is_var) {
      src_name = "cpnt_" + std::to_string (m_cpnt_num++);
      indent (os);
      print_start_component (os, src_name, get_constructor ("Double"));
      os << "(" << node->parent ()->build_name () << ", " << m_null_string
                << ", " << src << ");\n";
    }

    if (!ctrl->out ()->build_name ().empty ()) {
      dst = ctrl->out ()->build_name ();
    } else {
      dst = build_find (ctrl->out ()->get_path(), false);
    }
    if (dst.empty ()) {
      print_error_message (error_level::error,
                               "anonymous component in output of control node",
                               1);
    }
    build_properties(os);

    indent (os);    
    if (!node->name ().empty ()) {
      std::string new_name = "cpnt_" + std::to_string (m_cpnt_num++);
      m_parent_list.back ()->add_entry (node->name (), new_name);
      node->set_build_name (new_name);
      print_component_decl (os, new_name);
      os << " = ";
      node->set_name ("\"" + node->name () + "\"");
    } else {
      node->set_name (m_null_string);
    }

    print_component_constructor (os, constructor);
    os << " (" << node->parent ()->build_name () << ", " << node->name ();
    os << ", " << src << ", ";
    
    bool is_binding = node->djnn_type ().compare ("Binding") == 0;
    if (is_binding) {
      os << (ctrl->get_in_act () == "true" ? "ACTIVATION" : "DEACTIVATION" ) << ", " << dst
         << ", " << (ctrl->get_out_act () == "true" ? "ACTIVATION" : "DEACTIVATION" );
      used_processes["Binding"];
    } else {
      os << "\"\"" << ", ";
      os << dst << ", \"\"";

      bool is_assignment = node->djnn_type ().compare ("Assignment") == 0
        || node->djnn_type ().compare ("PausedAssignment") == 0
        || node->djnn_type ().compare ("LazyAssignment") == 0;

      if (is_assignment)
      {
        // is_model...
        os << ", " << node->args ().at (0).second;
      }
      
    }
    os << ");\n";
  }

  void
  CPPBuilder::build_multi_control_node (std::ostream &os,
                                           NativeExpressionNode *node)
  {
    emit_compiler_info(os);
    ExprNode* arg_node = node->get_expression ();
    std::string p_name =
            node->parent () == nullptr ? m_null_symbol : node->parent ()->build_name ();
    std::string arg = build_find (arg_node->get_path (), false);
    build_properties(os);
    std::string control_name = node->is_connector () ? "MultiConnector" : "MultiAssignment";
    used_processes[control_name] = true;
    int model = node->is_connector () ? !node->is_model () : node->is_model ();
    if (arg_node->get_path ()->has_wild_card ()) {
      for (auto e : node->get_output_nodes ()) {
        std::string out_arg = build_find (e, false);
        build_properties(os);
        indent (os);
        os << control_name << " (" << p_name << ", " << arg << ", " << out_arg
            << ", " << model << ");\n";
      }
      return;
    }
    indent (os);
    if (!m_fastcomp) {
      os << "{ djnnstl::vector <djnnstl::string> in_names;\n";
    } else {
      os << "{ const char* in_names [] = {\n";
    }
    std::vector<SubPathNode*> subpaths = arg_node->get_path()->get_subpath_list();
    std::string comma = "";
    for (auto p: subpaths.back()->get_path_list ()) {
      indent (os);
      if (!m_fastcomp) {
        os << "in_names.push_back (\"";
      } else {
        os << comma;
        comma = ",";
        os << "\"";
      }
      std::string sep = "";
      for (auto item: p->get_subpath_list()) {
        os << sep << item->get_subpath ();
        sep = "/";
      }
      if (!m_fastcomp) {
        os << "\");\n";
      } else {
        os << "\"" << "\n";
      }
    }
    if (m_fastcomp) {
      indent (os);
      os << "};\n";
    }
    m_indent++;
    for (auto e : node->get_output_nodes ()) {
      std::string out_arg = build_find (e, false);
      build_properties(os);
      indent (os);
      if (!m_fastcomp) {
      os << "{ djnnstl::vector <djnnstl::string> out_names;\n";
      } else {
        os << "{ const char* out_names [] = {\n";
      }
      comma = "";
      for (auto p: e->get_subpath_list().back()->get_path_list()) {
        indent (os);
        if (!m_fastcomp) {
          os << "out_names.push_back (\"";
        } else {
          os << comma;
          comma = ",";
          os << "\"";
        }
        std::string sep = "";
        for (auto item: p->get_subpath_list()) {
          os << sep << item->get_subpath ();
          sep = "/";
        }
        if (!m_fastcomp) {
          os << "\");\n";
        } else {
          os << "\"" << "\n";
        }
      }
      indent (os);
      if (m_fastcomp) {
        os << "};\n";
        indent (os);
      }
      
      os << control_name << " (" << p_name << ", "
         <<   arg << ", in_names, ";
      if (m_fastcomp)
        os << subpaths.back()->get_path_list ().size () << ", ";
      os << out_arg << ", out_names, ";
      if (m_fastcomp)
        os << e->get_subpath_list().back()->get_path_list().size () << ", ";
      os   << model << ");\n";
      
      indent (os);
      os << "}\n";
    }
    m_indent--;
    indent (os);
    os << "}\n";
  }

  void
  CPPBuilder::build_simple_control_node (std::ostream &os,
                                         NativeExpressionNode *node)
  {
    emit_compiler_info(os);
    std::string p_name =
        node->parent () == nullptr ? m_null_symbol : node->parent ()->build_name ();
    ExprNode *arg_node = node->get_expression ();
    std::string arg;
    bool templated = false;

    if (arg_node->get_expr_node_type () != PATH_EXPR) {
      std::string branch_name;
      if (node->parent()) {
        if (node->parent()->djnn_type().find("Switch")==0) {
          // if it's inside a switch, we should surround it with a component, or find the nearest parent component and put it there
          std::string branch_name = "cpnt_" + std::to_string (m_cpnt_num++);
          indent (os);
          os << "[[maybe_unused]] auto * " << branch_name << " = ";
          if (!m_fastcomp) {
            os << "new Component (";
          } else {
            os << "djnn_" << "new_Component (";
          }
          os << p_name << ",\"" <<  branch_name << ",\"); // constant in a component to make Switch* behave as expected\n";
          p_name = branch_name;
          used_processes["Component"] = true;
        }
      }
      
      std::string new_name ("cpnt_" + std::to_string (m_cpnt_num++));
      if (!branch_name.empty()) {
        arg = branch_name;
      } else {
        arg = new_name;
      }
  
      
      //std::cerr << "°° " << arg << " " << arg_node->get_val () << std::endl;
      ExprNode * en = dynamic_cast<ExprNode*>(arg_node);
      assert(en);

      //bool templated = (m_template_props.find(arg) != m_template_props.end());
      templated = en->get_expr_type() == DOUBLE_UNIT || en->get_expr_type() == INT_UNIT;

      indent (os);
      if (templated) {
        //os << "[[maybe_unused]] auto * " << new_name << " = new TemplateProperty<decltype(" << arg_node->get_val () << ")> (";
        os << "[[maybe_unused]] auto * " << new_name << " = new TemplateProperty (";
        used_processes["TemplateProperty"] = true;
      } else {
        if (arg_node->get_expr_type() == STRING) {
          if (!m_fastcomp) {
            os << "[[maybe_unused]] TextProperty * " << new_name << " = ";
          } else {
            os << "[[maybe_unused]] auto * " << new_name << " = ";
          }
          if (!m_fastcomp) {
            os << "new TextProperty (";
          } else {
            os << "djnn_" << "new_TextProperty (";
          }
          used_processes["TextProperty"] = true;
        } else {
          os << "[[maybe_unused]] auto * " << new_name << " = ";
          if (!m_fastcomp) {
            os << "new DoubleProperty (";
          } else {
            os << "djnn_" << "new_DoubleProperty (";
          }
          used_processes["DoubleProperty"] = true;
        }
      }

      os << p_name;
      os << ", \"\", " // empty name
         << arg_node->get_val ()
         << ");\n";

    } else {
      if (arg_node->get_path()->has_path_list() || arg_node->get_path()->has_wild_card()) {
        build_multi_control_node (os, node);
        return;
      }
      arg = build_find (arg_node->get_path (), false);
      build_properties(os);
    }

    if (!node->is_connector () && !node->name ().empty ()) {
      std::string new_name ("cpnt_" + std::to_string (m_cpnt_num++));
      std::string out_arg = build_find (node->get_output_nodes ().at (0), false);
      build_properties(os);
      indent (os);
      os << "[[maybe_unused]] auto * " << new_name << " = ";
      if (!m_fastcomp) {
        os << "new Assignment ( ";
      } else {
        os << "djnn_" << "new_Assignment ( ";
      }
      os
          << p_name
          << ", "
          << arg << ", "
          << out_arg << ", "
          << node->is_model () << ");\n";
      if (m_parent_list.back ()->add_entry (node->name (), new_name) == 1
          && node->duplicate_warning ())
        print_error_message (error_level::warning,
                             "duplicated name: " + node->name (), 0);
    } else {
      for (auto e : node->get_output_nodes ()) {
        std::string out_arg = build_find (e, false);
        build_properties(os);

        if(!templated)
          templated = (m_template_props.find(arg) != m_template_props.end());

        //std::cerr << "- " << arg << " " << out_arg << " " << templated << std::endl;
        string used_process_name;

        indent (os);
        if (!m_fastcomp) {
          os << "new ";
        } else {
          os << "djnn_" << "new_";
        }
        if (templated) {
          os << "T";
          used_process_name = "T";
        }

        if (node->is_paused ())
          os << "Paused";
        if (node->is_lazy ())
          os << "Lazy";

        if (node->is_connector ()) {
          os << "Connector";
          used_process_name += "Connector";
        }
        else {
          os << "Assignment";
          used_process_name += "Assignment";
        }

        used_processes[used_process_name] = true;

        if (templated) {
          os << "<";
          os << "std::remove_pointer<decltype(" << arg << ")>::type, ";
          os << "std::remove_pointer<decltype(" << out_arg << ")>::type";
          os << ">";
        }

        os << " (" << p_name << ", \"\", " <<   arg << ", " // << "\"\","
                                   << out_arg //<< ", \"\""
                                   ;
        // connectors don't have is_model but copy_on_activation so the meaning of this property is somewhat inverted
        if (!node->is_paused () && !node->is_lazy ()) {
          if(node->is_connector())
            os << ", " << !node->is_model ();
          else
            os << ", " << node->is_model ();
        }

        os << ");\n";
      }
    }
    build_properties(os);
  }


  static std::string remove_deref (std::string name)
  {
    if (name.compare (0, 22, ("(*((AbstractProperty*)")) == 0) {
      return name.substr(22, name.size() - 24);
    }
    return name;
  }

  void
  CPPBuilder::emit_not_a_property (std::ostream &os, const std::string& arg)
  {
    if (!m_fastcomp) {
      os << "if (" << arg << "->get_process_type() != PROPERTY_T) {\n";
    } else {
      os << "if (djnn_get_process_type (" << arg << ") != PROPERTY_T) {\n";
    }
    indent (os);
    os << "\tdjnn__error (" << arg << ", \"is not a property\\n\");\n";
    indent (os);
    os << "\tdjnn__exit(0);\n";
    indent (os);
    os << "}\n";
  }


  void
  CPPBuilder::build_native_expression_node (std::ostream &os, Node *n)
  {
    m_expr_in = m_expr_out = 0;
    NativeExpressionNode *node = dynamic_cast<NativeExpressionNode*> (n);
    if (node->get_expression ()->get_expr_node_type() < 2) {
      emit_compiler_info(os);
      build_simple_control_node (os, node);
      return;
    }

    std::string sym_name ("sym_" + std::to_string (m_sym_num++));
    std::string p_name = node->parent () == nullptr ? m_null_symbol : node->parent ()->build_name ();
    std::vector<std::string> triggers;

    std::string native_name_struct = node->get_build_name () + "_struct";
    std::string native_name_obj = node->get_build_name () + "_obj";

    std::string native_name ("cpnt_" + std::to_string (m_cpnt_num++));
    std::string n_expr_name = node->name ().empty () ? m_null_string : node->name ();

    ExprNode* expr = node->get_expression();
    std::vector <ExprNode*> leaves;
    extract_leaves (leaves, expr);

    vector<string> tmpl_varnames;
    // to avoid multiple loops, generate source code with multiple streams that we assemble at the end
    stringstream create_temp_properties;
    stringstream populate_native_fields;

    for (auto l : leaves) {
      if (l->get_expr_node_type() == PATH_EXPR) {
        auto & whatever_name = l->get_path ()->get_subpath_list().at (0)->get_subpath();
        string arg = build_find (l->get_path (), true);
        build_properties(create_temp_properties);
        //std::cerr << "--arg: " << arg << " --parent?_name: " << whatever_name << std::endl;
        if (arg.compare (0, 6, "d_var_") == 0 || arg.compare (0, 6, "i_var_") == 0) {
          string new_param_name = transform_name(whatever_name);
          if (prop_sym.find (new_param_name) == prop_sym.end ()) {
            emit_compiler_info(populate_native_fields);
            indent (populate_native_fields);
            populate_native_fields << native_name << "->" << new_param_name <<  "= " << arg
                  << ";\n";
          }

        } else if (arg.compare (0, 6, "s_var_") == 0) {
          string new_param_name = transform_name(whatever_name);
          if (prop_sym.find (new_param_name) == prop_sym.end ()) {
            emit_compiler_info(populate_native_fields);
            indent (populate_native_fields);
            populate_native_fields << native_name << "->"<< new_param_name <<  "= " << arg << ";\n";
          }

        } else {
          arg = remove_deref (arg);
          std::string fake_name = build_fake_name (l->get_path (), false);
          const char* str_to_find;
          if (!m_fastcomp) {
            str_to_find = "->";
          } else {
            str_to_find = "djnn_find";
          }
          //if (arg.find ("->") != std::string::npos) {
          //if (arg.find ("djnn_find") != std::string::npos) {
          if (arg.find (str_to_find) != std::string::npos) {
            std::string new_param_name = transform_name (fake_name);
            if (prop_sym.find (new_param_name) == prop_sym.end ()) {
              std::string tmpl_class_name = "AbstractProperty*";
              std::string new_name ("cpnt_" + std::to_string (m_cpnt_num++));

              emit_compiler_info(create_temp_properties);
              indent (create_temp_properties);
              emit_not_a_property (create_temp_properties, arg);
              indent (create_temp_properties);
              if (!m_fastcomp) {
                create_temp_properties << "[[maybe_unused]] auto * " << new_name << " = dynamic_cast<" << tmpl_class_name << "> (" << arg << ");\n\n";
              } else {
                create_temp_properties << "[[maybe_unused]] auto * " << new_name << " = " << arg << ";\n\n";
              }
              used_processes["AbstractProperty"] = true;
              sym[new_param_name] = new_name;
              prop_sym[new_param_name] = new_name;
            }
            const auto& new_name = prop_sym[new_param_name];
            std::string field_name = transform_name (fake_name);
            indent (populate_native_fields);
            populate_native_fields << native_name << "->" << field_name <<  " = " << new_name << ";\n";
            if (find(begin(tmpl_varnames), end(tmpl_varnames), new_name) == end(tmpl_varnames)) {
              triggers.push_back (new_name);
              tmpl_varnames.push_back (new_name);
            }

          } else {
            std::string new_param_name = transform_name(whatever_name);
            
            if (prop_sym.find (new_param_name) == prop_sym.end ()) {
              std::string new_name;     
              new_name = "cpnt_" + std::to_string (m_cpnt_num++);
              std::string tmpl_class_name = "AbstractProperty*";

              emit_compiler_info(create_temp_properties);
              indent (create_temp_properties);
              emit_not_a_property (create_temp_properties, arg);
              indent (create_temp_properties);
              if (!m_fastcomp) {
                create_temp_properties << "[[maybe_unused]] auto * " << new_name << " = dynamic_cast<" << tmpl_class_name << ">(" << arg << ");\n";
              } else {
                create_temp_properties << "[[maybe_unused]] auto * " << new_name << " = " << arg << ";\n";
              }
              used_processes["AbstractProperty"] = true;
              sym[new_param_name] = new_name;
              prop_sym[new_param_name] = new_name;

            }
            const auto& new_name = prop_sym[new_param_name];
            emit_compiler_info(populate_native_fields);
            indent (populate_native_fields);
            populate_native_fields << native_name << "->" << new_param_name << " = " 
              <<  new_name << ";\n";
            if (find(begin(tmpl_varnames), end(tmpl_varnames), new_name) == end(tmpl_varnames)) {
              triggers.push_back (new_name);
              tmpl_varnames.push_back(new_name);
            }
          }
        }
      }
    }

    for (auto e : node->get_output_nodes ()) {
      // whatever_name because I do not know the intent of this variable...
      auto & whatever_name = e->get_subpath_list().at (0)->get_subpath();
      std::string arg = build_find (e, false);
      build_properties(create_temp_properties);
      if (arg.compare (0, 4, "var_") != 0) {
        const char* str_to_find;
        if (!m_fastcomp) {
          str_to_find = "->";
        } else {
          str_to_find = "djnn_find";
        }
        if (arg.find (str_to_find) != std::string::npos) {
          std::string fake_name = build_fake_name (e, true);
          std::string new_param_name = transform_name (fake_name);

          if (prop_sym.find (new_param_name) == prop_sym.end ()) {
            std::string new_name ("cpnt_" + std::to_string (m_cpnt_num++));
            std::string tmpl_class_name = "AbstractProperty*";

            emit_compiler_info(create_temp_properties);
            indent (create_temp_properties);
            emit_not_a_property (create_temp_properties, arg);
            indent (create_temp_properties);
            if (!m_fastcomp) {
              create_temp_properties << "[[maybe_unused]] auto * " << new_name << " = dynamic_cast<" + tmpl_class_name + "> (" << arg << ");\n";
            } else {
              create_temp_properties << "[[maybe_unused]] auto * " << new_name << " = " << arg << ";\n";
            }

            used_processes["AbstractProperty"] = true;            
            sym[new_param_name] = new_name;
            prop_sym[new_param_name] = new_name;
          }
          const string& new_name = prop_sym[new_param_name];
          emit_compiler_info(populate_native_fields);
          indent (populate_native_fields);
          populate_native_fields << native_name << "->" << new_param_name << " = " << new_name << ";\n";
          if (find(begin(tmpl_varnames), end(tmpl_varnames), new_name) == end(tmpl_varnames)) {
            tmpl_varnames.push_back(new_name);
          }
          

        } else {
          std::string new_param_name = transform_name(whatever_name);
          std::string tmpl_class_name = "C_" + new_param_name;

          if (prop_sym.find (new_param_name) == prop_sym.end ()) {
            std::string new_name;
            new_name = "cpnt_" + std::to_string (m_cpnt_num++);
            std::string tmpl_class_name = "AbstractProperty*";

            emit_compiler_info(create_temp_properties);
            indent (create_temp_properties);
            emit_not_a_property (create_temp_properties, arg);
            indent (create_temp_properties);
            if (!m_fastcomp) {
              create_temp_properties << "[[maybe_unused]] auto * " << new_name << " = dynamic_cast<" << tmpl_class_name << ">(" << arg << ");\n";
            } else {
              create_temp_properties << "[[maybe_unused]] auto * " << new_name << " = " << arg << ";\n";
            }

            used_processes["AbstractProperty"] = true;
            prop_sym[new_param_name] = new_name;
          }
          const string& new_name = prop_sym[new_param_name];
          emit_compiler_info(populate_native_fields);
          indent (populate_native_fields);
          populate_native_fields << native_name << "->" << new_param_name << " = " <<  new_name << ";\n";
          
          if (find(begin(tmpl_varnames), end(tmpl_varnames), new_name) == end(tmpl_varnames)) {
            tmpl_varnames.push_back(new_name);
          }
        
        }
      }

    }

    stringstream native_stream;
    emit_compiler_info(native_stream);
    indent (native_stream);
    native_stream << "[[maybe_unused]] auto * " << native_name << " = new " << native_name_struct << "<";

    native_stream << "decltype(";

    std::copy(std::begin(tmpl_varnames),
              std::end(tmpl_varnames),
              std::experimental::make_ostream_joiner(native_stream, "), decltype("));

    native_stream << ")";
    native_stream << ">";
    native_stream << "("
       << p_name << ", " << n_expr_name << ", true, " << node->is_model ()
       << ");\n";

    // now that properties are built, call finalize_construction(), which in turn will call impl_activate()
    emit_compiler_info(populate_native_fields);
    indent (populate_native_fields);
    if (!m_fastcomp) {
      populate_native_fields << native_name << "->finalize_construction (";
    } else {
      populate_native_fields << "djnn_" << "finalize_construction (" << native_name << ", ";
    }
    populate_native_fields << p_name << ", " << n_expr_name << ");\n";

    emit_compiler_info(os);
    os << "// >>\n";
    os << create_temp_properties.str() << "\n";
    os << native_stream.str() << "\n";
    os << populate_native_fields.str() << "\n";

    std::string& new_name = native_name;

    if (!node->name ().empty ()) {
      if (m_parent_list.back ()->add_entry (node->name (), new_name) == 1
          && node->duplicate_warning ())
        print_error_message (error_level::warning,
                             "duplicated name: " + node->name (), 0);
    }
    std::string native_edge_name = new_name;
    if (node->is_connector ()) {
      emit_compiler_info(os);
      std::string sync_name ("cpnt_" + std::to_string (m_cpnt_num++));
      indent (os);
      if (!m_fastcomp) {
        os << "Synchronizer* " << sync_name << " = new Synchronizer (" << p_name;
      } else {
        os << "auto * " << sync_name << " = djnn_new_Synchronizer (" << p_name;
      }
      os << ", \"sync_"+sync_name+"\", " << new_name << ", \"\"); //FIXME remove Synchronizer\n"; // FIXME
      used_processes["Synchronizer"]=true;
      native_edge_name = sync_name;
      for (auto t : triggers) {
        indent (os);
        if (!m_fastcomp) {
          os << sync_name << "->add_source (" << t << ", \"\");\n";
        } else {
          os << "djnn_synchronizer_add_source (" << sync_name << ", " << t << ", \"\");\n";
        }
      }
    }
    emit_compiler_info(os);
    for (auto out : node->get_output_nodes ()) {
      std::string arg = build_find (out, false);
      build_properties(create_temp_properties);
      indent (os);
      if (!m_fastcomp) {
        os << native_edge_name <<"->add_native_edge (" << new_name << "," ;
      } else {
        os << "djnn_add_native_edge (" << native_edge_name << ", " << new_name << ", " ;
      }
      os << arg << ");\n";
    }

    os << "//<<\n\n";
  }

  void
  CPPBuilder::build_native_action (std::ostream &os, Node *n)
  {
    emit_compiler_info(os);
    NativeActionNode *node = dynamic_cast<NativeActionNode*> (n);
    os << "void\n";
    os << node->action_name () << " (CoreProcess *" << node->param_name () << ")\n";
    const std::string code = node->code ();
    if (code[0] != '{') {
      os << "{\n";
    }
    os << code;
    if (code[code.length () - 1] != '}') {
      os << "}";
    }
    os << std::endl;
  }

  void
  CPPBuilder::build_native_collection_action (std::ostream &os, Node *n)
  {
    emit_compiler_info(os);
    NativeCollectionActionNode *node = dynamic_cast<NativeCollectionActionNode*> (n);
    os << "static void\n";
    os << node->action_name () << " (CoreProcess *" << node->param_name () << ", const djnnstl::vector<CoreProcess*> " << node->list_name() << ")\n"; // FIXME shoudl be a ref
    const std::string code = node->code ();
    if (code[0] != '{') {
      os << "{\n";
    }
    os << code;
    if (code[code.length () - 1] != '}') {
      os << "}";
    }
    os << std::endl;
  }

  void
  CPPBuilder::build_native_expression (std::ostream &os, Node *n)
  {
    emit_compiler_info(os);
    NativeExpressionNode *node = dynamic_cast<NativeExpressionNode*> (n);
    if (node->get_expression ()->get_expr_node_type() < STEP) {
      return;
    }
    std::string unique_name = m_filename;
    std::replace (unique_name.begin (), unique_name.end (), '/', '_');
    std::replace (unique_name.begin (), unique_name.end (), '.', '_');
    std::replace (unique_name.begin (), unique_name.end (), '-', '_');

    std::string native_name ("nat_" + unique_name + "_" + std::to_string (m_native_num++));
    node->set_build_name (native_name);

    // first pass: generate nat struct
    std::map<std::string,bool> already_handled;
    std::string native_name_struct = native_name + "_struct";
    std::string native_name_obj = native_name + "_obj";

    std::ostringstream struct_stream;

    vector<string> tmpl_types;

    struct_stream << "struct " << native_name_struct;
    if (!m_fastcomp) {
      struct_stream << " : public NativeExpressionAction {\n";
      struct_stream << "\t" << native_name_struct << " (djnn::CoreProcess *p, ";
      struct_stream << "const djnnstl::string &n";
      struct_stream << ", bool string_setter, bool is_model=false)";
      struct_stream << R"(
       : NativeExpressionAction (p, n, string_setter)
    {
      set_is_model (is_model);
    }
      // note: finalize_construction will call impl_activate before proper properties are fully built, leading to a crash
      // so delay finalize_construction after building properties
      // finalize_construction (p, n);
    )";
    } else {
      struct_stream << " : public NativeExpressionActionProxy {\n";
      struct_stream << "\t" << native_name_struct << " (djnn::CoreProcess *p, ";
      struct_stream << "const char *n";
      struct_stream << ", bool string_setter, bool is_model=false)";
      struct_stream << R"(
      : NativeExpressionActionProxy (p, n, string_setter, is_model) {}
    )";
    }
    struct_stream << R"(
    bool _string_setter;
    void impl_activate () override;)" << "\n";
    //struct_stream << "\t" << "void impl_deactivate () override {}\n)";

    m_expr_in = m_expr_out = 0;

    ExprNode* expr = node->get_expression();
    std::vector <ExprNode*> leaves;
    extract_leaves (leaves, expr);

    for (auto l : leaves) {
      if (l->get_expr_node_type () == PATH_EXPR) {
        std::string tn = transform_name(build_fake_name (l->get_path (), false));
        if( ! already_handled.count(tn) ) {
          switch (l->get_expr_type()) {
            case BOOL:
            case INT:
              struct_stream << "\tint ";
              break;
            case DOUBLE:
              struct_stream << "\tdouble ";
              break;
            case STRING:
              struct_stream << "\tdjnnstl::string ";
              break;
            case CAST_STRING:
            case PROCESS:
              // struct_stream << "\tusing C_" << tn << "_t = " << "C_" << tn << ";\n";
              struct_stream << "\tC_" << tn << " ";
              tmpl_types.push_back(string("C_") + tn);
              //std::cout << l->get_val() << std::endl;
              break;
            default:
              print_error_message (error_level::error, "Incorrect type in expression", 1);
              break;
          }
          struct_stream << tn << ";\n";
          already_handled[tn] = true;
        }
      }
    }

    for (auto n : node->get_output_nodes ()) {
      std::string tn = transform_name(build_fake_name(n, true));
      if( ! already_handled.count(tn) ) {
        //struct_stream << "\tusing C_" << tn << "_t = " << "C_" << tn << ";\n";
        struct_stream << "\tC_" << tn << " " << tn << ";\n";
        tmpl_types.push_back(string("C_") + tn);
        already_handled[tn] = true;
      }
    }

    std::ostringstream struct_template_stream;
    struct_template_stream << "typename ";

    std::copy(std::begin(tmpl_types),
              std::end(tmpl_types),
              std::experimental::make_ostream_joiner(struct_template_stream, ", typename "));

    os << "\n\n";
    os << "template <" << struct_template_stream.str() << ">\n";
    
    struct_stream << "};\n";
    os << struct_stream.str() << "\n";

    // second pass: generate expression
    m_expr_in = m_expr_out = 0;
    os << "template <" << struct_template_stream.str() << ">\n";
    os << "void\n" << native_name_struct;
    os << "<";
    std::copy(std::begin(tmpl_types),
              std::end(tmpl_types),
              std::experimental::make_ostream_joiner(os, ", "));
    os << ">";
    os << "::impl_activate ()\n{\n";
    for (auto n : node->get_output_nodes ()) {
      os << "\t";
      if (!m_fastcomp) {
        os << transform_name (build_fake_name(n, true)) << "->set_value (";
      } else {
        os << "djnn_" << "set_value (" <<  transform_name (build_fake_name(n, true)) << ",";
      }

      ExprNode* expr = node->get_expression();
      os << build_expr (expr, undefined_t, true); // this should NOT create a new dynamic_casted property !!!
      os << ", " << !node->is_paused () << ");\n";
    }
    indent (os);
    os << "};\n\n";

    used_processes["NativeExpressionAction"] = true;
  }

  void
  CPPBuilder::build_end_define (std::ostream &os, Node *node)
  {
    indent (os);
    os << "return " << m_parent_list.back ()->get_symbol ("this") << ";\n}\n";
    m_indent--;
    // BuildNode* n;
    // n = m_parent_list.at (m_parent_list.size () - 1);
    // m_parent_list.pop_back ();
    // if (n)
    //   delete n;
    // // FIXME why twice ?!!!
    // n = m_parent_list.at (m_parent_list.size () - 1);
    // m_parent_list.pop_back ();
    // if (n)
    //   delete n;
  }

  void
  CPPBuilder::build_instruction (std::ostream &os, Node *node)
  {
    emit_compiler_info(os);
    InstructionNode *n = dynamic_cast<InstructionNode*> (node);
    for (size_t i = 0; i < n->path_list ().size (); i++) {
      std::string arg = build_find (n->path_list ().at (i), false);
      if (arg.empty ()) {
        print_error_message (error_level::error,
         "unknown component " + n->path_list ().at (i)->get_subpath_list ().at (0)->get_subpath(), 1);
        return;
      }
      build_properties(os);
      indent (os);
      switch (n->type ()) {
        case DUMP:
          os << "if (" << arg << ")" << endl ;
          m_indent += 1;
          indent (os);
          if (!m_fastcomp)
            os << arg << "->dump (0);\n";
          else
            os << "djnn_dump (" << arg << ")\n";
          m_indent -= 1;
          //indent (os);
          //s << "else" << endl ;
          //indent (os); indent (os);
          //os << "cout <<  endl << endl << \"warning - dump could not resolve: \" << " << arg <<  " << endl << endl;" << endl;
          break;
        case XMLSERIALIZE:
          os << "if (" << arg << ")" << endl ;
          m_indent += 1;
          indent (os);
          if (!m_fastcomp)
            os << arg << "->serialize (\"XML\");\n";
          else
            os << "djnn_serialize (" << arg << ")\n";
          m_indent -= 1;
          //indent (os);
          //os << "else" << endl ;
          //indent (os); indent (os);
          //os << "cout <<  endl << endl << \"warning - XMLSerialize could not resolve: \" << " << arg <<  " << endl << endl;" << endl;
          break;
        case NOTIFY:
          if (!m_fastcomp)
            os << arg << "->notify_activation ();\n";
          else
            os << "djnn_notify_activation (" << arg << ");\n";
          break;
        case RUN: {
          auto& ml = n->path_list ().at (i)->get_subpath_list().at(0)->get_subpath();
          if (ml.compare("syshook") == 0 || ml.compare("mainloop") == 0) {
            if (!m_fastcomp)
              os << "MainLoop::instance ().activate ();\n";
            else
              os << "djnn_activate (djnn_mainloop_instance ()); \n";
          } else {
            if (!m_fastcomp)
              os << arg << "->activate ();\n";
            else
              os << "djnn_activate (" << arg << ");\n";
          }
          }
          break;
        case STOP: {
          auto& ml = n->path_list ().at (i)->get_subpath_list().at(0)->get_subpath();
          if (ml.compare("syshook") == 0 || ml.compare("mainloop") == 0) {
            if (!m_fastcomp)
              os << "MainLoop::instance ().deactivate ();\n";
            else
              os << "djnn_deactivate (djnn_mainloop_instance ()); \n";
          } else {
            if (!m_fastcomp)
              os << arg << "->deactivate ();\n";
            else
              os << "djnn_deactivate (" << arg << ");\n";
          }
          }
          break;
        case DELETE: {
          /* 
            Be Careful !
            in case of modification this code is replicated in DELETE CONTENT
          */
          std::string new_name ("cpnt_" + std::to_string (m_cpnt_num++));
          os << "[[maybe_unused]] auto * " << new_name << " = " << arg << ";\n";
          indent (os);
          os << "if (" << new_name << ") {\n";
          m_indent += 1;
          indent (os);
          os << "auto& cpnt = " << new_name << ";\n";
          indent (os);
          if (! m_fastcomp) {
            os << R"(
      cpnt->deactivate ();
      if (cpnt->get_parent ())
        cpnt->get_parent ()->remove_child (dynamic_cast<CoreProcess*>(cpnt));
      cpnt->schedule_deletion ();
      cpnt = nullptr;
    )";
          } else {
            os << R"(
      djnn_deactivate (cpnt);
      if (djnn_get_parent (cpnt))
        djnn_remove_child (djnn_get_parent (cpnt), cpnt);
      djnn_schedule_deletion (cpnt);
      cpnt = nullptr;
    )";

          }


          // m_indent += 1;
          // indent (os);
          // os << new_name << "->deactivate ();\n";
          // indent (os);
          // os << "if (" << new_name << "->get_parent ())\n";
          // m_indent += 1;
          // indent (os);
          // os << new_name << "->get_parent ()->remove_child (dynamic_cast<CoreProcess*>(" << new_name << "));\n";
          // m_indent -= 1;
          // indent (os);
          // os << new_name << "->schedule_deletion ();\n";
          // indent (os);
          // os << new_name << " = nullptr;\n";
          m_indent -= 1;
          indent (os);
          os << "}\n";
          }
          break;
        case DELETE_CONTENT: {
          std::string new_layer_name ("is_layer_" + std::to_string (m_cpnt_num++));
          os << "Layer *" << new_layer_name << " = dynamic_cast<Layer *> (" << arg << ");\n";
          indent (os); os << "if (" << new_layer_name << ") {\n";
          
          indent (os); indent (os); os << "puts (\"\\nERROR - delete_content should not be used on Layer (better use a component inside a Layer\\n\");\n";
          indent (os); indent (os); os << "djnn__exit(0);\n";
          indent (os); os << "}\n";
          std::string new_container_name ("cpnt_" + std::to_string (m_cpnt_num++));
          indent (os); os << "Container *" << new_container_name << " = dynamic_cast<Container *> (" << arg << ");\n";
          indent (os); os << "if (" << new_container_name << ") {\n";

          /*
            note:
            We DO NOT use Container->clean_up_content () anymore
            because the delete in clean_up_content are not scheduled  
            instead use the same code as DELETE (just above)
            old code:
            indent (os); indent (os); os << new_name << "->clean_up_content ();\n";
          */
          
          std::string new_container_size (new_container_name + "_size");
          indent (os); indent (os); os << "int "<< new_container_size << " = " << new_container_name << "->children ().size ();\n";
          indent (os); indent (os); os << "for (int i = " << new_container_size << " - 1; i >= 0; i--) {\n";
          /* replicate of DELETE */
          std::string new_child_name ("cpnt_" + std::to_string (m_cpnt_num++));
          indent (os); indent (os); indent (os); os << "[[maybe_unused]] auto * " << new_child_name << " = " << new_container_name << "->children ()[i];\n";
          indent (os); indent (os); indent (os); os << "if (" << new_child_name << ") {\n";
          indent (os); indent (os); indent (os); indent (os); os << new_child_name << "->deactivate ();\n";
          indent (os); indent (os); indent (os); indent (os); os << "if (" << new_child_name << "->get_parent ())\n";
          indent (os); indent (os); indent (os); indent (os); indent (os); os << new_child_name << "->get_parent ()->remove_child (dynamic_cast<CoreProcess*>(" << new_child_name << "));\n";
          indent (os); indent (os); indent (os); indent (os); os << new_child_name << "->schedule_deletion ();\n";
          indent (os); indent (os); indent (os); indent (os); os << new_child_name << " = nullptr;\n";
          indent (os); indent (os); indent (os); os << "}\n";
          /* end replicate of DELETE */
          indent (os); indent (os); os << "}\n";
          indent (os); os << "}\n";
          indent (os); os << "else {\n";
          indent (os); indent (os); os << "puts (\"\\nERROR - delete_content should be used on Containers (except Layer)\\n\");\n";
          indent (os); indent (os); os << "djnn__exit(0);\n";
          indent (os); os << "}\n";

          used_processes["Layer"] = true;
        }
        break;
        case UNKNOWN:
          print_error_message (error_level::error,
          "unknown instruction " + n->path_list ().at (i)->get_subpath_list ().at (0)->get_subpath(), 1);
          break;
    }
  }
}

  void
  CPPBuilder::set_property (std::ostream &os, Node *node)
  {
    emit_compiler_info(os);
    auto expr_str = build_expr (node->get_args().at(0), undefined_t);
    build_properties(os);

    std::string prop_name = node->get_path()->get_subpath_list().at(0)->get_subpath();
    std::string found = m_parent_list.back ()->get_symbol (prop_name);
    // DBG;
    // for (auto & pp: m_parent_list) { std::cerr << pp->name() << " * ";}
    // std::cerr << std::endl;
    // if the symbol is unknown we take it as the definition of a new Process*
    if (found.empty ()) {
      std::string var_name ("cpnt_" + std::to_string (m_cpnt_num++));
      if (m_parent_list.back ()->add_entry (prop_name, var_name) == 1
                && node->duplicate_warning ())
              print_error_message (error_level::warning,
                                   "duplicated name: " + node->name (), 0);
      if (!m_in_for)
        indent (os);
      os << "[[maybe_unused]] auto * " << var_name << " = " << expr_str << ";\n";
      sym[prop_name] = var_name;
      prop_sym[prop_name] = var_name;
    } else {
      prop_name = build_find (node->get_path(), false);
      build_properties(os);
      if (!m_in_for)
        indent (os);
      //std::cerr << prop_name << " " << " " << sizeof("djnn_find (") << " " << prop_name[12] << " " << prop_name.find ("cpnt_", sizeof("djnn_find (")) << std::endl;
      bool cond;
      //if (!m_fastcomp) {
        cond = (prop_name.find ("cpnt_", 0) == 0) || (prop_name.find ("arg_", 0)) == 0;
      //} else {
        cond |= (prop_name.find ("cpnt_", sizeof("djnn_find (")-1) != string::npos) || (prop_name.find ("arg_", sizeof("djnn_find (")-1) != string::npos);
      //}
      if (cond) {
        if (!m_fastcomp) {
          os << "((AbstractProperty*) "<< prop_name << ")->set_value (" << expr_str << ", true)";
        } else {
          os << "djnn_set_value ("<< prop_name << ", " << expr_str << ", true)";
        }
        if (!m_in_for)
          os << ";\n";
        used_processes["AbstractProperty"] = true;
      } else {
        os << prop_name << " = " << expr_str;
        if (!m_in_for)
          os << ";\n";
      }
    }
  }

  void
  CPPBuilder::self_set_property (std::ostream &os, Node *n)
  {
    emit_compiler_info(os);
    SelfAssignNode *node = (SelfAssignNode*)n;
    std::string prop_name = node->get_path()->get_subpath_list().at(0)->get_subpath();
    std::string found = m_parent_list.back ()->get_symbol (prop_name);
    std::string symbol = (std::string) node->symbol ();
    prop_name = build_find (node->get_path(), false);
    build_properties (os);
    auto expr_str = build_expr (node->get_args().at(0), undefined_t);
    build_properties (os);

    bool cond;
    if (!m_fastcomp) {
      cond = (prop_name.find ("cpnt_", 0) == 0) || (prop_name.find ("arg_", 0) == 0);
    } else {
      cond = (prop_name.find ("cpnt_", sizeof("djnn_find (")-1) != string::npos) || (prop_name.find ("arg_", sizeof("djnn_find (")-1) != string::npos);
    }
    if (cond) {
      used_processes["AbstractProperty"] = true;
      used_processes["TextProperty"] = true;

      string new_name = "cpnt_" + std::to_string (m_cpnt_num++);
      indent (os);
      os << "[[maybe_unused]] auto * " << new_name << " = ";
      if (!m_fastcomp) {
        os << "dynamic_cast<AbstractProperty*>(" << prop_name << ");\n";
      } else {
        os << "(" << prop_name << ");\n";
      }
      sym[prop_name] = new_name;
      prop_sym[prop_name] = new_name;

      if (!m_in_for) {
        indent (os);
        os << "if (";
        if (!m_fastcomp) {
          os << new_name << "->get_prop_type() == String";
        } else {
          os << "djnn_get_process_type(" << new_name << " ) == String";
        }
        os << ") {\n";
        os << "\t\tdjnn__warning (" << prop_name << ", \"invalid operand for String Property\");\n";
        os << "\t}";
        os << "else {\n";
        os << "\t\t";
        if (!m_fastcomp) {
        os << new_name << "->set_value (";
        } else {
          os << "djnn_set_value (" << new_name ;
        }
        if (!m_fastcomp) {
          os << new_name << "->get_double_value () " << symbol;
        } else {
          os << ", djnn_get_double_value (" << new_name << ") " << symbol;
        }
        os << " " <<  expr_str << ", true);\n";
        os << "\t}";
      } else {
        if (!m_fastcomp) {
          os << new_name << "->set_value (";
        } else {
          os << "djnn_set_value (" << new_name << ")" ;
        }
        if (!m_fastcomp) {
          os << new_name << "->get_double_value () " << symbol;
        } else {
          os << ", djnn_get_double_value (" << new_name << ") " << symbol;
        }
        os << " " << expr_str << ", true)";
      }
    } else {
      if (!m_in_for)
        indent (os);
      os << prop_name << " " << symbol << "= " << expr_str;
      if (!m_in_for)
        os << ";\n";
    }
  }

  void
  CPPBuilder::alias (std::ostream &os, Node *node)
  {
    emit_compiler_info(os);
    BinaryInstructionNode *n = dynamic_cast<BinaryInstructionNode*> (node);
    std::string arg = build_find (n->right_arg (), false);
    build_properties (os);
    std::string new_name ("cpnt_" + std::to_string (m_cpnt_num++));
    const std::string& whatever_name = n->left_arg ()->get_subpath_list().at(0)->get_subpath();
    indent (os);
    os << "alias (" << m_parent_list.back ()->name () << ", \""
        << whatever_name << "\", ";
    if (!m_fastcomp) {
      os << "dynamic_cast<CoreProcess*>(" << arg << "));\n";
    } else {
      os << "(" << arg << "));\n";
    }
    indent (os);
    os << "[[maybe_unused]] auto * " << new_name << " = ";
    if (!m_fastcomp) {
      os << m_parent_list.back ()->name () << "->find_child (";
    } else {
      os << "djnn_find (" << m_parent_list.back ()->name () << ", ";
    }
    os << "\"" << whatever_name + "\");\n";
    if (m_parent_list.back ()->add_entry (whatever_name, new_name) == 1
        && node->duplicate_warning ())
      print_error_message (error_level::warning,
                           "duplicated name: " + whatever_name, 0);
  }

  void
  CPPBuilder::merge (std::ostream &os, Node *node)
  {
    emit_compiler_info(os);
    BinaryInstructionNode *n = dynamic_cast<BinaryInstructionNode*> (node);
    std::string left = build_find (n->left_arg (), false);
    std::string right = build_find (n->right_arg (), false);
    build_properties (os);
    indent (os);
    os << "merge_children (" << left << ", \"\", "
        << right << ", \"\");\n";
  }

  void
  CPPBuilder::remove (std::ostream &os, Node *node)
  {
    emit_compiler_info(os);
    BinaryInstructionNode *n = dynamic_cast<BinaryInstructionNode*> (node);
    std::string left = build_find (n->left_arg (), false);
    std::string right = build_find (n->right_arg (), false);
    build_properties (os);
    indent (os);
    os << left << "->remove_child ( dynamic_cast<CoreProcess*>(" << right << "));\n";
  }

  void
  CPPBuilder::move (std::ostream &os, Node *node, const string &c)
  {
    emit_compiler_info(os);
    BinaryInstructionNode *n = dynamic_cast<BinaryInstructionNode*> (node);
    std::string last;
    std::string left = build_find (n->left_arg (), false);
    build_properties (os);
    if (n->right_arg ()) {
      std::string last = build_find (n->right_arg (), false);
      build_properties (os);
      indent (os);
      if (!m_fastcomp) {
        os << left << "->get_parent ()->move_child (dynamic_cast<CoreProcess*> (";
      } else {
        os << "djnn_move_child((djnn_get_parent (" << left << ")), (";
      }
      os << left << "), "
              << c << ", " << last << ");\n";
    }
    else {
      indent (os);
      if (!m_fastcomp) {
        os << left << "->get_parent ()->move_child (dynamic_cast<CoreProcess*>(";
      } else {
        os << "djnn_move_child((djnn_get_parent (" << left << ")), (";
      }
      os << left << "), "
        << c << ", nullptr);\n";
    }
  }

  void
  CPPBuilder::add_child (std::ostream &os, Node *node)
  {
    emit_compiler_info(os);
    std::string new_name ("cpnt_" + std::to_string (m_cpnt_num++));
    if (!node->name().empty() && !node->keep_name ()) {
      if (m_parent_list.back ()->add_entry (node->name (), new_name) == 1) {
        print_error_message (error_level::warning,
                             "duplicated name: " + node->name (), 0);
      }
    }
    m_cur_building_name = new_name;

    auto expr_str = build_expr (node->get_args().at (0), process_t);
    build_properties (os);
    indent (os);
    os << "[[maybe_unused]] auto * " << new_name << " = " << expr_str;
  }

  void
  CPPBuilder::build_end_add_child (std::ostream &os, Node *n)
  {
    os << ";\n";
    emit_compiler_info(os);
    indent (os);
    if (!m_fastcomp) {
      os << m_parent_list.back ()->name () << "->add_child (";
    } else {
      os << "djnn_add_child (" << m_parent_list.back ()->name () << ", ";
    }
    os << m_cur_building_name;
    if (n->keep_name ()) {
      os << ", " << m_parent_list.back ()->get_symbol (n->name());
    } else {
      os << ", \""
        << m_parent_list.back ()->get_key (m_cur_building_name) << "\"";}
    os << ");\n";
  }

  void
  CPPBuilder::fetch_add_child (std::ostream &os, const std::string &parent,
                               const std::string &child, const std::string &name)
  {
    emit_compiler_info(os);
    if (parent == m_null_symbol)
      return;
    indent (os);
    if (!m_fastcomp) {
      os << parent << "->add_child";
    } else {
      os << "djnn_add_child (" << parent << ",";
    }
    if (!m_fastcomp) {
      os << " (dynamic_cast<CoreProcess*>(" << child << ")";
    } else {
      os << " (" << child << ")";
    }
    os << ", \"" << name << "\");\n";
  }

  void
  CPPBuilder::add_children_to (std::ostream &os, Node *node)
  {
    std::string new_name ("cpnt_" + std::to_string (m_cpnt_num++));
    std::string s = build_find (node->get_path(), false);
    build_properties (os);

    AddChildrenToNode* add_children_to = dynamic_cast<AddChildrenToNode*> (node);
    if (!add_children_to->children ().empty()) {
      for (auto name : add_children_to->children ()) {
        auto child = build_find (name, false);
        build_properties (os);
        emit_compiler_info(os);
        indent (os);
        if (!m_fastcomp) {
          os << s << "->add_child (";
        } else {
          os << "djnn_add_child (" << s << ", ";
        }
        os << child << ",\"" << name->get_subpath_list ().back()->get_subpath () << "\");\n";
      }
      //m_parent_list.push_back (new BuildNode (s, m_parent_list.back ()));
      push_ctxt (s);
      /* FIXME dirty trick to set the parent name of the enclosed nodes*/
      node->set_build_name (s);
      return;
    }
    if (node->get_path()->get_subpath_list().size ()> 1) {
      //m_parent_list.push_back (new BuildNode (new_name, m_parent_list.back ()));
      push_ctxt (new_name);
      /* FIXME dirty trick to set the parent name of the enclosed nodes*/
      node->set_build_name (new_name);

      emit_compiler_info(os);
      indent (os);
      os << "[[maybe_unused]] auto * " << new_name << " = " << s << ";\n";     
      indent (os);
      os << "if (" << new_name << " == nullptr)\n";
      m_indent += 1;
      indent (os);
      os << "djnn__exit (1);\n";
      m_indent -=1;
      //os << "\tcerr <<  endl << endl << \"ERROR - processing addChildrenTo - the component \\\"\" << \"" << node->name () << "\\\"\" << \" is null or do not exist yet\" << endl << endl;\n";
      //os << "error " ... // djnn-cpp should raise an error
      
    } else {
      m_parent_list.push_back (new BuildNode (s, m_parent_list.back ()));
      push_ctxt (s);
      /* FIXME dirty trick to set the parent name of the enclosed nodes*/
      node->set_build_name (s);
    }
  }

  void
  CPPBuilder::build_this_node (std::ostream &os, Node *node)
  {
    emit_compiler_info(os);
    ThisNode* th = dynamic_cast<ThisNode*>(node);
    std::string new_name ("cpnt_" + std::to_string (m_cpnt_num++));
    th->set_build_name (new_name);
    if (m_parent_list.back ()->add_entry ("this", new_name) == 1)
      print_error_message (error_level::warning,
                           "duplicated name: " + node->name (), 0);
    if (th->inherit()) {
      ExprNode* e = th->get_super_class ();
      std::string args_str;
      for (auto sub : ((FunctionExprNode*)e)->get_args()) {
        args_str += ", " + build_expr(sub);
      }
      build_properties(os);

      indent (os);
      os << "[[maybe_unused]] auto * " << new_name << " = ";
      
      std::string expr =  e->get_val () + " (p, n";
      used_processes[e->get_val ()] = true;
      expr += args_str;
      expr += ");\n";

      os  << expr;
    } else {
      indent (os);
      os << "[[maybe_unused]] auto * " << new_name << " = ";
      if (!m_fastcomp) {
        os << "new Component (p, n);\n";
      } else {
        os << "djnn_" <<  "new_Component (p, n);\n";
      }
    }
    used_processes["Component"] = true;

    /* We make the hypothesis that "this" is the first node after _define_ thus
     * the symbols in the sym_table should be the arguments of the function.
     * They must be duplicated to check for symbol duplication */
    BuildNode *new_parent = new BuildNode (new_name, m_parent_list.back ());
    new_parent->sym_table ()->insert (
        m_parent_list.back ()->sym_table ()->begin (),
        m_parent_list.back ()->sym_table ()->end ());
    m_parent_list.push_back (new_parent);
  }

  void
  CPPBuilder::build_define_node (std::ostream &os, Node *node)
  { //DBG;
    emit_compiler_info(os);

    // since we start a new cpp function, all of our previously created variables are not valid anymore
    // so clear everything
    sym.clear();
    m_new_syms_from_build_expr.clear();
    prop_sym.clear();
    
    //m_parent_list.push_back (new BuildNode ("", m_parent_list.back ()));

    os << "CoreProcess*\n" << node->name () << " (CoreProcess *p, ";
    if (!m_fastcomp) {
      os << "const djnnstl::string &n";
    } else {
      os << "const char * n";
    }
    std::vector< named_parameter_t > data = node->get_args_spec();
    for (size_t j = 0; j < data.size (); j++) {
      named_parameter_t arg = data.at (j);
      os << ", ";
      print_type (os, arg.first);
      std::string new_name;
      switch (arg.first.first) {
        case INT:
        case BOOL:
          new_name = "i_var_" + std::to_string (m_var_num++);
          break;
        case DOUBLE:
          new_name = "d_var_" + std::to_string (m_var_num++);
          break;
        case STRING:
          new_name = "s_var_" + std::to_string (m_var_num++);
          break;
        case NATIVE_CODE_T:
          used_processes["NativeCode"] = true;
        case LOCAL_NAME:
        case NAME:
        case PROCESS:
          new_name = "arg_" + std::to_string (m_cpnt_num++);
          break;
        case INT_ARRAY:
        case DOUBLE_ARRAY:
        case STRING_ARRAY:
        case PROCESS_ARRAY:
          new_name = "array_var_" + std::to_string (m_var_num++);
          break;
        default:;
      }
      os << " " << new_name;
      if (m_parent_list.back ()->add_entry (arg.second, new_name) == 1)
        print_error_message (error_level::warning,
                             "duplicated name: " + arg.second, 0);
    }
    os << ")\n{\n";
    m_indent++;
  }

  void
  CPPBuilder::build_main_node (std::ostream &os)
  {
    emit_compiler_info(os);
    /* main */
    os << "int\nmain (int argc, char* argv[]) {\n";
    m_indent = 1;
    int size = m_ast.preamble ().use ().size ();
    bool has_display = false;

    emit_compiler_info(os);
  
    /* init modules from use */
    for (int i = 0; i < size; ++i) {
      std::string str = m_ast.preamble ().use ().at (i);
      if (str == "display") {
        if (!has_display) {
          has_display = true;
        }
        else {
          continue;
        }
      }
      if (str == "gui" && !has_display) {
        indent (os);
        if (m_fastcomp)
          os << "djnn_";
        os << "init_display ();\n"; // do it before init_gui ()
        has_display = true;
      }
      /* add corresponding init_MODULE */
      indent (os);
      if (m_fastcomp)
        os << "djnn_";
      os << "init_" << str << " ();\n";

      if (str == "core") {
        indent (os);
        if (m_fastcomp)
          os << "djnn_";
        os << "init_exec_env ();\n"; // do it after init_core ()
      }
    }
  }

  void
  CPPBuilder::build_end_main (std::ostream &os, Node *node)
  {
    emit_compiler_info(os);
    Node* data = (Node*) node->get_user_data ();
    if (data == nullptr)
      return;
    indent (os);
    if (!m_fastcomp) {
      os << data->build_name () << "->activate ();\n";
    } else {
      os << "djnn_" << "activate (" << data->build_name () << ");\n";
    }
    indent (os);
    if (!m_fastcomp) {
      os << "MainLoop::instance ().activate ();\n\n";
    } else {
      os << "djnn_activate (djnn_mainloop_instance());\n";
    }
    used_processes["MainLoop"] = true;

    if (m_cleaner) {
      indent (os);
      if (!m_fastcomp) {
        os << data->build_name () << "->deactivate ();\n";
        indent (os);
        os << "delete " << data->build_name () << ";\n";
      } else {
        os << "djnn_" << "deactivate (" << data->build_name () << ");\n";
        indent (os);
        os << "djnn_" << "delete (" << data->build_name () << ");\n";
      }

      emit_compiler_info(os);
      /* clear modules from use */
      int size = m_ast.preamble ().use ().size ();
      bool has_display = false;
      for (int i = 0; i < size; ++i) {
        const std::string str = m_ast.preamble ().use ().at (i);
        if (str == "display") {
          has_display = true; 
        }
      }

      for (int i = size-1; i >= 0; --i) {
        const std::string str = m_ast.preamble ().use ().at (i);
        
        /* add corresponding clear_MODULE */
        if (str == "core") {
          indent (os);
          if (m_fastcomp)
            os << "djnn_";
          os << "clear_exec_env ();\n"; // do it after init_core ()
        }

        indent (os);
        if (m_fastcomp)
          os << "djnn_";
        os << "clear_" << str << " ();\n";

        if (str == "gui" && !has_display) {
          indent (os);
          if (m_fastcomp)
            os << "djnn_";
          os << "clear_display ();\n"; // do it after clear_gui ()
        }
      }
    }
  }

  void
  CPPBuilder::build_native_action_component (std::ostream &os, Node *n)
  {
    emit_compiler_info(os);
    NativeComponentNode* node = dynamic_cast<NativeComponentNode*> (n);
    native_type type = node->get_native_type();
    std::string constructor;
    switch (type) {
      case SIMPLE_ACTION:
        constructor = "NativeAction";
        break;
      case ASYNC_ACTION:
        constructor = "NativeAsyncAction";
        break;
      case COLLECTION_ACTION:
        constructor = "NativeCollectionAction";
    }
    used_processes[constructor] = true;
    std::string name =
        node->name ().empty () ? m_null_string : "\"" + node->name () + "\"";
    std::string new_name ("cpnt_" + std::to_string (m_cpnt_num++));
    node->set_build_name (new_name);
    if (!node->name ().empty ()) {
      if (m_parent_list.back ()->add_entry (node->name (), new_name) == 1)
        print_error_message (error_level::warning,
                             "duplicated name: " + node->name (), 0);
    }
    std::string function_name = m_parent_list.back ()->get_symbol (node->function_name ());
    if (function_name.empty ())
      function_name = node->function_name ();
    std::string p_name =
        node->parent () == nullptr ? m_null_symbol : node->parent ()->build_name ();
    
    std::string list;
    if (type == COLLECTION_ACTION) {
      list = build_find (node->path_list(), false);
      build_properties(os);
    }
    std::string data;
    if (node->path_data() == nullptr)
      data = m_null_symbol;
    else {
      data = build_find(node->path_data(), false);
      build_properties(os);
    }

    indent (os);
    os << "[[maybe_unused]] auto * " << new_name << " = ";
    if (!m_fastcomp) {
      os << "new " << constructor;
    } else {
      os << "djnn_new_" << constructor;
    }
    os << " (" << p_name
        << ", " << name << ", " << function_name << ", ";
    if (type == COLLECTION_ACTION) {
      os << list << ", ";
    }
    os << data << ", " << node->is_model () << ");\n ";
  }

  void
  CPPBuilder::build_transition_node (std::ostream &os, Node *n)
  {
    emit_compiler_info(os);
    TransitionNode* ctrl = dynamic_cast<TransitionNode*> (n);
    std::string constructor = get_constructor (ctrl->djnn_type ());
    indent (os);
    if (!m_fastcomp) {
      os << "new " << constructor;
    } else {
      os << "djnn_" << "new_" << constructor;
    }
    os << " (" << m_parent_list.back ()->get_symbol (ctrl->parent ()->name())
        << ", " << "\"" << ctrl->name () << "\"";

    std::string src, dst;
    src = m_parent_list.back ()->get_symbol (ctrl->get_src ());
    dst = m_parent_list.back ()->get_symbol (ctrl->get_dst ());
    os << ", " << src << ", " << dst << ", ";
    std::string trigger = build_find (ctrl->get_trigger(), false);
    os << trigger << ", \"\", ";
    if (ctrl->get_action()) {
      os << build_find (ctrl->get_action(), false) << ", \"\");\n";
    } else {
      os << "nullptr, \"\");\n";
    }
  }

  void
  CPPBuilder::build_smala_native (std::ostream &os, Node *node)
  {
    emit_compiler_info(os);

    SmalaNative *n = dynamic_cast<SmalaNative*> (node);

    std::string src_name = "cpnt_" + std::to_string (m_cpnt_num++);
    std::string data_name = "cpnt_" + std::to_string (m_cpnt_num++);

    os << "\nstatic void\n" << n->fct () << " (CoreProcess* c) {\n";
    os << "\t[[maybe_unused]] auto * " << src_name << " = ";
    if (!m_fastcomp) {
      os << "c->get_activation_source ();\n";
    } else {
      os << "djnn_get_activation_source (c);\n";
    }
    os << "\t[[maybe_unused]] Process * " << data_name << " = ";
    if (!m_fastcomp) {
      os << "(Process *) get_native_user_data (c);\n";
    } else {
      os << "(Process*) djnn_get_native_user_data (c);\n";
    }

    if (m_parent_list.back ()->add_entry (n->src (), src_name) == 1) {
      print_error_message (error_level::warning,
                           "duplicated name: " + n->src (), 0);
    }

    int sz = n->data ()->get_subpath_list ().size();
    std::string user_data_name = sz >= 1 ? n->data ()->get_subpath_list ().at (sz - 1)->get_subpath () : "no_user_data";
    if (m_parent_list.back ()->add_entry (user_data_name, data_name) == 1) {
      print_error_message (error_level::warning,
                           "duplicated name: " + user_data_name, 0);
    }

    m_indent = 1;

    //m_parent_list.push_back (new BuildNode ("0", m_parent_list.back ()));
    push_ctxt ("0"); //DBG;
    // since we start a new cpp function, none of our previously created variables are valid anymore
    // so clear everything
    sym.clear();
    m_new_syms_from_build_expr.clear();
    prop_sym.clear();
    m_template_props.clear();
  }

  void
  CPPBuilder::build_end_native (std::ostream &os)
  {
    emit_compiler_info(os);

    pop_ctxt(); //DBG;
    m_indent = 0;
    os << "}\n\n";
  }

  void
  CPPBuilder::build_header (const std::string &prefix)
  {
    std::ofstream os (prefix + ".h");
    std::string s = prefix;
    std::replace (s.begin (), s.end (), '/', '_');
    for (size_t i = 0; i < prefix.length (); i++)
      s.at (i) = std::toupper (s.at (i));
    //os << "#pragma once\n#include <string>\n\n";
    os << "#pragma once\n\n";

    emit_compiler_info(os);

    bool set_include = false;
    for (auto def: m_ast.define_node_list ()) {
      if (def->include_native () && !set_include) {
        os << "#include \"core/control/native_action.h\"\n\n";
        set_include = true;
      }
      os << "djnn::CoreProcess* " << def->name ()
          << " (djnn::CoreProcess*, ";
      if (!m_fastcomp) {
        os << "const djnnstl::string &";
      } else {
        os << "const char*";
      }
      for (size_t j = 0; j < def->args ().size (); j++) {
        named_parameter_t arg = def->args ().at (j);
        os << ", ";
        print_type (os, arg.first);
      }
      os << ");\n";
    }
    os.close ();
  }

  void
  CPPBuilder::build_causal_dep (std::ostream &os, Node* node)
  {
    emit_compiler_info(os);
    CausalDependencyNode* n = dynamic_cast<CausalDependencyNode*> (node);
    std::string src = build_find (n->src (), true);
    std::string dst = build_find (n->dst (), true);
    build_properties(os);
    std::string p_name = (node->parent () == nullptr || node->ignore_parent ()) ? m_null_symbol : node->parent ()->build_name ();
    indent (os);
    if (!m_fastcomp) {
      os << "new GraphEdgeAdder (" << p_name << ", \"\", " << src << ", " << dst << ");\n";
    } else {
      os << "djnn_" << "new_GraphEdgeAdder (" << p_name << ", \"\", " << src << ", " << dst << ");\n";
    }
    used_processes["GraphEdgeAdder"] = true;
  }

  static
  void print_array_type (std::ostream &os, const std::string& type, int dimensions)
  {
    if (!m_fastcomp) {
      for (int n = 0; n < dimensions; n++) {
        os << "djnnstl::vector< ";
      }
    } else {
      os << type;
    }
    for (int n = 0; n < dimensions; n++) {
      if (!m_fastcomp) {
        os << " >";
      } else {
        os << "[]";
      }
    }
  }

  void
  CPPBuilder::print_type (std::ostream &os, parameter_t type, ExprNode* arg)
  {
    switch (type.first) {
      case INT: {
        os << "[[maybe_unused]] int";
        break;
      }
      case DOUBLE: {
        os << "[[maybe_unused]] double";
        break;
      }
      case STRING: {
        if (!m_fastcomp) {
          os << "[[maybe_unused]] const djnnstl::string&";
        } else {
          os << "[[maybe_unused]] const mystring&";
          //os << "[[maybe_unused]] auto";
          //os << "[[maybe_unused]] const my_string&";
        }
        break;
      }
      case NATIVE_CODE_T: {
        os << "[[maybe_unused]] djnn::NativeCode*";
        break;
      }
      case NAME:
      case PROCESS:
      {
        os << "[[maybe_unused]] djnn::CoreProcess*";
        break;
      }
      case DOUBLE_ARRAY:
      {
        print_array_type (os, "double", type.second);
        break;
      }
      case INT_ARRAY:
      {
        print_array_type (os, "int", type.second);
        break;
      }
      case STRING_ARRAY:
      {
        print_array_type (os, "string", type.second);
        break;
      }
      case PROCESS_ARRAY:
      {
        print_array_type (os, "Process*", type.second);
        break;
      }
      default:
        break;
    }
  }

  void 
  CPPBuilder::print_items_array (std::ostream &os, ArrayItemsNode* n)
  {
    std::string pre = "";
    os << "{";
    for (auto it: n->get_items()) {
      os << pre;
      pre = ", ";
      switch (it->get_expr_node_type ()) {
        case LITERAL:
          os << it->get_val ();
          break;
        case PATH_EXPR:
          os << build_find (it->get_path (), true);
          break;
        case ARRAY:
        {
          ArrayItemsNode* array = dynamic_cast<ArrayItemsNode*> (it);
          if (array)
            print_items_array (os, array);
          break;
        }
        default:;
      }
    }
    os << "}";
  }
  void 
  CPPBuilder::build_array_var (std::ostream &os, ArrayVarNode* a) 
  {
    emit_compiler_info(os);
    indent (os);
    if (!m_fastcomp) {
      for (int n = 0; n < a->get_dimension (); n++) {
        os << "djnnstl::vector< ";
      }
    }
    switch (a->get_array_type ()) {
      case DOUBLE:
        os << "double";
        break;
      case INT:
        os << "int";
        break;
      case STRING:
        os << "string";
        break;
      case PROCESS:
        os << "Process*";
        break;
      default:;
    }
    if (!m_fastcomp) {
      for (int n = 0; n < a->get_dimension (); n++) {
        os << " >";
      }
    }
    std::string new_name = "a_var_" + std::to_string (m_cpnt_num++);
    if (m_parent_list.back ()->add_entry (a->get_name (), new_name) == 1 && a->duplicate_warning ())
      print_error_message (error_level::warning, "duplicated name: " + a->get_name (), 0);
    os << " " << new_name;
    if (m_fastcomp) {
      for (int n = 0; n < a->get_dimension (); n++) {
        os << "[]";
      }
    }
    os << " = ";
    print_items_array (os, a->get_array ());
    os << ";\n";
  }

  void
  CPPBuilder::print_component_decl (std::ostream &os, const std::string &name)
  {
    //emit_compiler_info(os);
    os << "[[maybe_unused]] auto * " << name;
  }

  void
  CPPBuilder::print_component_constructor (std::ostream &os,
                                           const std::string &constructor)
  {
    //emit_compiler_info(os);
    std::map<std::string, std::string>::iterator it;
    it = m_import_types.find (constructor);
    if (it == m_import_types.end ()) {
      if (!m_fastcomp) {
        os << "new " << constructor;
      } else {
        os << "djnn_" << "new_" << constructor; // c header
      }
    }
    else {
      // it's an import, hence we call a function that creates a Component
      os << constructor;
    }
  }
} /* namespace Smala */
