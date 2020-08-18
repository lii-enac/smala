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

#include <locale>
#include <algorithm>

namespace Smala
{

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

  int
  CPPBuilder::build (const Ast &ast, const std::string &builddir,
                     const std::string &prefix)
  {
    m_indent = 0;
    m_cpnt_num = 0;
    m_var_num = 0;
    m_ast = ast;
    m_types.clear ();
    m_parent_list.clear ();
    m_parent_list.push_back (new BuildNode (m_null_symbol)); // the first parent is null
    if (!ast.is_main ())
      build_define (prefix);
    m_filename = std::string (prefix) + ".cpp";
    std::ofstream os (prefix + ".cpp");
    os << "#include <iostream>\n";
    os << "#include <string>\n";
    os << "#include \"core/core-dev.h\"\n";
    os << "#include \"exec_env/exec_env.h\"\n\n";
    os << "using namespace std;\nusing namespace djnn;\n\n";

    os << "#include \"core/utils/error.h\" // for Context class\n";
    os << "#undef error // avoid name clash with error macro and possible following #include\n";
    os << "#undef warning // avoid name clash with error macro and possible following #include\n\n";

    int size = m_ast.preamble ().import ().size ();
    for (int i = 0; i < size; ++i) {
      /* add the import name to the possible types */
      std::string name = m_ast.preamble ().import ().at (i);
      m_types.insert (std::pair<std::string, std::string> (name, name));
      m_import_types.insert (std::pair<std::string, std::string> (name, name));
    }

    build_preamble (os);

    size = m_ast.node_list ().size ();
    for (int i = 0; i < size; ++i) {
      build_node (os, m_ast.node_list ().at (i));
    }
    if (m_ast.is_main ())
      os << "}\n";
    os.close ();
    return m_error;
  }

  void
  CPPBuilder::build_use (std::ofstream &os, std::string use)
  {
    if (use.compare ("core") == 0)
      return;
    else if (use.compare ("gui") == 0) {
      if (!m_display_initialized) {
        os << "#include \"display/display.h\"\n";
        m_display_initialized = true;
      }
    } else if (use.compare ("display") == 0) {
      if (m_display_initialized)
        return;
      m_display_initialized = true;
    }
    os << "#include \"" << use << "/" << use << ".h\"\n";
  }

  void
  CPPBuilder::build_import (std::ofstream &os, Node *n)
  {
    os << "#include \"";
    auto sep = "/";
    auto *sep_2 ="";
    for (auto sub : n->get_path()->get_subpath_list()) {
      os << sep_2 << sub->get_subpath();
      sep_2 = sep;
    }
    os << ".h\"\n";
  }

  std::string
  CPPBuilder::build_fake_name (PathNode* n, bool out)
  {
    std::string fake = n->get_subpath_list ().at (0)->get_subpath();
    for (int i = 1; i < n->get_subpath_list ().size (); i++) {
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
  CPPBuilder::build_term_str (TermNode *term)
  {
    switch (term->arg_type ()) {
      case SYMBOL:
      case OPERATOR:
      case VALUE:
      case FUNCTION_CALL:
      case STRING_VALUE: {
        return term->str_arg_value ();
        break;
      }
      case VAR: {
        std::string p = build_find (term->path_arg_value (), false);
        // if the name contains "var_" then this is a simple variable not a djnn property
        // so write it as is
        std::size_t found = p.find ("var_");
        return p;
        break;
      }
      case SMALA_NULL: {
        return m_null_symbol;
        break;
      }
      default:
        return "";
    }
  }

  std::string
  CPPBuilder::build_path (PathNode* n)
  {
    std::vector<SubPathNode*> n_list = n->get_subpath_list ();
    std::string symbol = n_list.at (0)->get_subpath ();
    std::string str = m_parent_list.back ()->get_symbol (symbol);
    if (n_list.size () == 1)
      return str;
    str += "->find_child (\"";
    std::string pref = "";
    for (int i = 1; i < n_list.size (); i++) {
      str += pref;
      if (n_list.at (i)->get_path_type () != EXPR)
        str += n->get_subpath_list ().at (i)->get_subpath ();
      else {
        std::vector <TermNode*> terms = n->get_subpath_list ().at (i)->get_expr ();
        str += build_term_str (terms.at (0));
       }
     pref = "/";
    }
    str += "\")";
    return str;
  }

  static bool
  has_complex_term (PathNode* n)
  {
    std::vector<SubPathNode*> n_list = n->get_subpath_list ();
    for (auto n: n_list) {
      if (n->get_path_type () == EXPR) {
        std::vector <TermNode*> terms = n->get_expr ();
        if (terms.size () > 1)
          return true;
        else if (terms.at (0)->arg_type() != VALUE)
          return true;
      }
    }
    return false;
  }

  std::string
  CPPBuilder::build_find (PathNode* n, bool ignore_cast)
  {
    std::vector<SubPathNode*> n_list = n->get_subpath_list ();
    if (n_list.empty ())
      return "";

    std::string str;
    std::string prefix;
    std::string symbol = n_list.at (0)->get_subpath ();
    bool complex_term = has_complex_term (n);

    /* check for complex expression in terms, if not build classic path*/
    if (!complex_term)
      str = build_path (n);
    else
      str = m_parent_list.back ()->get_symbol (symbol);


    if (!ignore_cast) {
      switch (n->get_cast ()) {
        case NO_CAST:
        case BY_PROCESS:
          prefix = "";
          break;
        case BY_VALUE:
          prefix = "((AbstractProperty*)";
          break;
        case BY_REF:
          prefix = "(*((AbstractProperty*)";
          break;
      }
    }
    if (str.empty ()) {
      // then check if it is a Djnn symbol that is a key prefixed by DJN
      if (symbol.substr (0, 3) == "DJN")
        return symbol;
      else {
        // finally check if it is a Smala symbol
        str = m_type_manager->get_smala_symbol (symbol);
        if (!str.empty ())
          return str;
      }
      // if everything fails, print an error message
      print_error_message (error_level::error, "Symbol not found: " + symbol,
                           1);
      return symbol;
    }
    if (str.compare (0, 6, "d_var_") == 0
        || str.compare (0, 6, "i_var_") == 0
        || str.compare (0, 6, "s_var_") == 0
        || str.compare (0, 4, "var_") == 0) {
      return str;
    }
    if (complex_term) {
      for (auto it = n_list.begin() + 1; it != n_list.end(); ++it) {
        if ((*it)->get_path_type () != EXPR)
          str += "->find_child (\"" + (*it)->get_subpath () + "\")";
        else {
          str += "->find_child (";
          for (auto term : (*it)->get_expr ()) {
            str += build_term_str (term);
          }
          str += ")";
        }
      }
    }
    if (!ignore_cast) {
      switch (n->get_cast ()) {
        case BY_VALUE:
          str += ")->get_double_value ()";
          break;
        case BY_REF:
          str += "))";
          break;
        default:
          break;
      }
    }
  return prefix + str;
}

  void
  CPPBuilder::build_step (std::ofstream &os, Node *node, bool is_incr)
  {
    if (!m_in_for)
      indent (os);
    std::string find = build_find (node->get_path(), false);
    if (find.rfind ("cpnt_", 0) == 0) {
      os << "((AbstractProperty*) " << find << ")->set_value (";
      os << "((AbstractProperty*) " << find << ")->get_double_value ()";
      if (is_incr)
        os << " +";
      else
        os << " - ";
      os << " 1, true)";
    } else {
      os << find;
      if (is_incr)
        os << "++";
      else
        os << "--";
    }
    if (!m_in_for)
      os << ";\n";
  }

  void
  CPPBuilder::build_print (std::ofstream &os, Node *node)
  {
    indent (os);
    std::string name ("var_" + std::to_string (m_var_num++));
    os << "std::string " << name << " ( ";
    std::vector<TermNode*> expr = node->get_expr_data ();
    for (auto cur: expr) {
      TermNode *n = static_cast<TermNode*> (cur);
      switch (n->arg_type ()) {
        case SYMBOL:
        case OPERATOR: {
          if (n->str_arg_value() != "+")
            print_error_message (error_level::error, "only + symbol is allowed in string expression", 1);
          os << " " << n->str_arg_value () << " ";
        }
          break;
        case VALUE: {
          os << n->str_arg_value ();
        }
          break;
        case STRING_VALUE:
          os << n->str_arg_value ();
          break;
        case VAR: {
          std::string p = build_find (n->path_arg_value (), false);
          // if the name contains "var_" then this is a simple variable not a djnn property
          // so write it as is and return
          std::size_t found = p.find ("var_");
          if (found != std::string::npos) {
            os << p;
            return;
          }
          os << "((AbstractProperty*)" << p << ")->get_string_value ()";
          break;
        }
        default:
          return;
      }
    }
    os << ");\n";
    indent (os);
    os << "cout << " << name << ";\n";
  }

  void
  CPPBuilder::build_while (std::ofstream &os, Node *node)
  {
    indent (os);
    os << "while (";
    m_in_static_expr = true;
    for (auto cur: node->get_expr_data ()) {
      build_node (os, cur);
    }
    m_in_static_expr = false;
    os << ") {\n";
    push_ctxt ();
    m_indent++;
  }

  void
  CPPBuilder::build_for (std::ofstream &os, Node *node)
  {
    indent (os);
    os << "for ";
    push_ctxt ();
    m_in_for = true;
  }

  void
  CPPBuilder::build_for_every (std::ofstream &os, Node *node)
  {
    ForEveryNode* n = (ForEveryNode*) node;
    indent (os);
    std::string list_name = "list_" + std::to_string (m_cpnt_num++);
    std::string loc_name = "cpnt_" + std::to_string (m_cpnt_num++);
    os << "std::vector<CoreProcess*> " << list_name << ";\n";
    indent (os);
    std::string path = build_find (n->get_path (), true);
    os << "auto *" << loc_name << " = " << path << ";\n";
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
    os << "std::cerr << \"Error: only Container and ProcessCollector can be used in forevery instruction\\n\";\n";
    indent (os);
    os << "exit (0);\n";
    m_indent--;
    indent (os);
    os << "}\n";
    indent (os);
    std::string var_name = "cpnt_" + std::to_string (m_cpnt_num++);
    os << "for (auto " << var_name << " : " << list_name << ") {\n";
    m_indent++;
    push_ctxt ();
    m_parent_list.back ()->add_entry (n->get_new_name (), var_name);
  }

  void
  CPPBuilder::pop_ctxt ()
  {
    BuildNode* n = m_parent_list.at (m_parent_list.size() - 1);
    m_parent_list.pop_back ();
    if (n) delete n;
  }

  void
  CPPBuilder::push_ctxt ()
  {
    m_parent_list.push_back (new BuildNode (m_parent_list.back ()->name (), m_parent_list.back ()));
  }

  void
  CPPBuilder::build_control_node (std::ofstream &os, Node *node)
  {
    CtrlNode *ctrl = static_cast<CtrlNode*> (node);
    std::string constructor = get_constructor (node->djnn_type ());
    bool is_binding = node->djnn_type ().compare ("Binding") == 0;

    std::string src = build_find (ctrl->in ()->get_path(), false);
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
    } else
      dst = build_find (ctrl->out ()->get_path(), false);
    if (dst.empty ()) {
      print_error_message (error_level::error,
                               "anonymous component in output of control node",
                               1);
    }

    if (is_binding) {
      /* check for synchronizer */
      indent (os);
      os << "if (dynamic_cast<Synchronizer*> (" << dst << ") != nullptr) {\n";
      indent (os);
      os << "\t((Synchronizer*) " << dst << ")->add_source (" << src << ", \"\");\n";
      indent (os);
      os << "} else {\n";
      m_indent++;
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

      //print_component_constructor (os, constructor);
      indent (os);
      os << "new Binding ";
      os << " (" << node->parent ()->build_name () << ", " << node->name ();
      os << ", " << src << ", "; //\"\", ";
      os << (ctrl->get_in_act () == "true" ? "ACTIVATION" : "DEACTIVATION" )<< ", " << dst //<< ", \"\""
          << ", " << (ctrl->get_out_act () == "true" ? "ACTIVATION" : "DEACTIVATION" ) << ");\n";
      m_indent--;
      indent (os);
      os << "}\n";
    } else {
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
      os << ", " << src << ", \"\"" << ", ";
      os << dst << ", \"\"";
      if (node->djnn_type ().compare ("Assignment") == 0
          || node->djnn_type ().compare ("PausedAssignment") == 0) {
        os << ", " << node->args ().at (0).second;
      }
      os << ");\n";
    }
  }

  void
  CPPBuilder::build_simple_control_node (std::ofstream &os,
                                         NativeExpressionNode *node)
  {
    std::string p_name =
        node->parent () == nullptr ? m_null_symbol : node->parent ()->build_name ();
    TermNode* arg_node = node->get_expression ().at (0);
    std::string arg;
    if (arg_node->arg_type () != VAR) {
      std::string new_name ("cpnt_" + std::to_string (m_cpnt_num++));
      indent (os);
      if (arg_node->str_arg_value ().at (0) == '\"') {
        os << "TextProperty *" << new_name << " = new TextProperty (" << p_name
            << ", \"\", " << arg_node->str_arg_value () << ");\n";
      } else {
        os << "DoubleProperty *" << new_name << " = new DoubleProperty ("
            << p_name << ", \"\", " << arg_node->str_arg_value () << ");\n";
      }
      arg = new_name;
    } else {
      arg = build_find (arg_node->path_arg_value (), false);
    }
    if (!node->is_connector () && !node->name ().empty ()) {
      std::string new_name ("cpnt_" + std::to_string (m_cpnt_num++));
      std::string out_arg = build_find (node->get_output_nodes ().at (0), false);
      os << "auto *" << new_name << " = new Assignment ( " << p_name
          << ", "//\"\", "
          << arg << ", "//\"\","
          << out_arg << ", "//\"\", "
          << node->is_model () << ");\n";
      if (m_parent_list.back ()->add_entry (node->name (), new_name) == 1
          && node->duplicate_warning ())
        print_error_message (error_level::warning,
                             "duplicated name: " + node->name (), 0);
    } else {
      for (auto e : node->get_output_nodes ()) {
        indent (os);
        std::string out_arg = build_find (e, false);
        os << "new ";
        if (node->is_paused ())
          os << "Paused";
        if (node->is_connector ())
          os << "Connector (";
        else
          os << "Assignment (";
        os << p_name << ", \"\", " <<   arg << ", " // << "\"\","
                                   << out_arg //<< ", \"\""
                                   ;
        // connectors don't have is_model but copy_on_activation so the meaning of this property is somewhat inverted
        if (!node->is_paused ()) {
          if(node->is_connector())
          os << ", " << !node->is_model ();
            else
          os << ", " << node->is_model ();
        }

        os << ");\n";
      }
    }
  }


  std::string
  transform_name(const std::string& input)
  {
    std::string new_param_name = input;
    std::replace(new_param_name.begin(), new_param_name.end(), '.','_');
    std::replace(new_param_name.begin(), new_param_name.end(), '/','_');
    std::replace(new_param_name.begin(), new_param_name.end(), '-','_');
    new_param_name.erase(std::remove(new_param_name.begin(), new_param_name.end(), '"'), new_param_name.end());
    return new_param_name;
  }

  static
  std::string remove_deref (std::string name)
  {
    if (name.compare (0, 22, ("(*((AbstractProperty*)")) == 0) {
      return name.substr(22, name.size() - 24);
    }
    return name;
  }

  void
  CPPBuilder::build_native_expression_node (std::ofstream &os, Node *n)
  {
    m_expr_in = m_expr_out = 0;
    NativeExpressionNode *node = static_cast<NativeExpressionNode*> (n);
    if (node->get_expression ().size () == 1) {
      build_simple_control_node (os, node);
      return;
    }
    std::map<std::string, std::string> sym;
    std::string sym_name ("sym_" + std::to_string (m_sym_num++));
    std::string p_name =
        node->parent () == nullptr ? m_null_symbol : node->parent ()->build_name ();
    std::vector<std::string> triggers;
    indent (os);


    std::string native_name_struct = node->get_build_name () + "_struct";
    std::string native_name_obj = node->get_build_name () + "_obj";


    std::string native_name ("cpnt_" + std::to_string (m_cpnt_num++));
    std::string n_expr_name =
        node->name ().empty () ? m_null_string : node->name ();
    os << "auto *" << native_name << " = new "<< native_name_struct << " ("
       << p_name << ", " << n_expr_name << ", true, " << node->is_model ()
       << ");\n";

    for (auto e : node->get_expression ()) {
      if ((e->arg_type () == VAR)
          && sym.find (e->path_arg_value ()->get_subpath_list().at (0)->get_subpath()) == sym.end ()) {
        std::string arg = build_find (e->path_arg_value(), true);

        if (arg.compare (0, 6, "d_var_") == 0
            || arg.compare (0, 6, "i_var_") == 0) {
          indent (os);
          std::string new_name ("cpnt_" + std::to_string (m_cpnt_num++));
          os << "DoubleProperty *" << new_name << " = new DoubleProperty ("
              << p_name << ", \"\", " << arg << ");\n";
          indent (os);
          std::string new_param_name = transform_name(e->path_arg_value ()->get_subpath_list().at (0)->get_subpath());
            os << native_name << "->"<< new_param_name <<  "= " << new_name
                << ";\n";

        } else if (arg.compare (0, 6, "s_var_") == 0) {
          indent (os);
          std::string new_name ("cpnt_" + std::to_string (m_cpnt_num++));
          os << "TextProperty *" << new_name << " = new TextProperty ("
              << p_name << ", \"\", " <<arg << ");\n";
          indent (os);
          std::string new_param_name = transform_name(e->path_arg_value ()->get_subpath_list().at (0)->get_subpath());
            os << native_name << "->"<< new_param_name <<  "= " << new_name
                << ";\n";

        } else {
          indent (os);
          arg = remove_deref (arg);
          if (arg.find ("->") != std::string::npos) {
            std::string new_name ("cpnt_" + std::to_string (m_cpnt_num++));
            os << "AbstractProperty* " << new_name
                << " = dynamic_cast<AbstractProperty*> (" << arg << ");\n";
            indent (os);
            os << "if (" << new_name << " == nullptr) {\n";
            indent (os);
            os << "\tcerr << \"" << e->path_arg_value ()->get_subpath_list().at (0)->get_subpath()
                << "\" << \" is not a property\\n\";\n";
            indent (os);
            os << "\texit(0);\n";
            indent (os);
            os << "}\n";
            indent (os);
            std::string fake_name = build_fake_name (e->path_arg_value (), false);
            std::string new_param_name = transform_name (fake_name);
            os << native_name << "->"<< new_param_name <<  "= " << new_name
                << ";\n";
            sym[fake_name] = new_name;
            triggers.push_back (new_name);

          } else {
            os << "if (dynamic_cast<AbstractProperty*>(" << arg
                << ") == nullptr) {\n";
            indent (os);
            os << "\tcerr << \"" << e->path_arg_value ()->get_subpath_list().at (0)->get_subpath()
                << "\" << \" is not a property\\n\";\n";
            indent (os);
            os << "\texit(0);\n";
            indent (os);
            os << "}\n";
            indent (os);
            std::string new_param_name = transform_name(e->path_arg_value ()->get_subpath_list().at (0)->get_subpath());
            os << native_name << "->"<< new_param_name <<  " = dynamic_cast<AbstractProperty*>(" << arg
                << ");\n";
            sym[e->path_arg_value ()->get_subpath_list().at (0)->get_subpath()] = arg;
            triggers.push_back (arg);
            
          }
        }
      }
    }

    for (auto e : node->get_output_nodes ()) {
      if (sym.find (e->get_subpath_list().at (0)->get_subpath()) == sym.end ()) {
        std::string arg = build_find (e, false);
        if (arg.compare (0, 4, "var_") != 0) {
          indent (os);
          if (arg.find ("->") == std::string::npos) {
            os << "if (dynamic_cast<AbstractProperty*>(" << arg
                << ") == nullptr) {\n";
            indent (os);
            os << "\tcerr << \"" << e << "\" << \" is not a property\\n\";\n";
            indent (os);
            os << "\texit(0);\n";
            indent (os);
            os << "}\n";
            indent (os);
            std::string new_param_name = transform_name(e->get_subpath_list().at (0)->get_subpath());
            os << native_name << "->"<< new_param_name
                << " = dynamic_cast<AbstractProperty*>(" << arg
                << ");\n";
            sym[e->get_subpath_list().at (0)->get_subpath()] = arg;
          } else {
            std::string new_name ("cpnt_" + std::to_string (m_cpnt_num++));
            os << "AbstractProperty* " << new_name
                << " = dynamic_cast<AbstractProperty*> (" << arg << ");\n";
            indent (os);
            os << "if (" << new_name << " == nullptr) {\n";
            indent (os);
            os << "\tcerr << \"" << e << "\" << \" is not a property\\n\";\n";
            indent (os);
            os << "\texit(0);\n";
            indent (os);
            os << "}\n";
            indent (os);
            std::string fake_name = build_fake_name (e, true);
            std::string new_param_name = transform_name (fake_name);
            os << native_name << "->" << new_param_name << " = " << new_name << ";\n";
            sym[fake_name] = new_name;
          }
        }
      }
    }

    indent (os);
    // now that properties are built, call finalize_construction, which in turn will call impl_activate
    os << native_name << "->finalize_construction (" << p_name << ", " << n_expr_name << ");\n";

    std::string& new_name = native_name;

    if (!node->name ().empty ()) {
      if (m_parent_list.back ()->add_entry (node->name (), new_name) == 1
          && node->duplicate_warning ())
        print_error_message (error_level::warning,
                             "duplicated name: " + node->name (), 0);
    }
    std::string native_edge_name = new_name;
    if (node->is_connector ()) {
      indent (os);
      std::string sync_name ("cpnt_" + std::to_string (m_cpnt_num++));
      os << "Synchronizer* " << sync_name << " = new Synchronizer (" << p_name
          << ", \"sync_"+sync_name+"\", " << new_name << ", \"\");\n";
      native_edge_name = sync_name;
      for (auto t : triggers) {
        indent (os);
        os << sync_name << "->add_source (" << t << ", \"\");\n";
      }
    }
    for (auto out : node->get_output_nodes ()) {
      std::string arg = build_find (out, false);
      indent (os);
      os << native_edge_name <<"->add_native_edge (" << new_name << "," ;
      os << arg << ");\n";
    }
  }

  void
  CPPBuilder::build_native_action (std::ofstream &os, Node *n)
  {
    NativeActionNode *node = static_cast<NativeActionNode*> (n);
    os << "void\n";
    os << node->action_name () << "(CoreProcess *" << node->param_name () << ")\n";
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
  CPPBuilder::build_native_collection_action (std::ofstream &os, Node *n)
  {
    NativeCollectionActionNode *node = static_cast<NativeCollectionActionNode*> (n);
    os << "static void\n";
    os << node->action_name () << "(CoreProcess *" << node->param_name () << ", std::vector<CoreProcess*> " << node->list_name() << ")\n";
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
  CPPBuilder::build_native_expression (std::ofstream &os, Node *n)
  {
    NativeExpressionNode *node = static_cast<NativeExpressionNode*> (n);
    if (node->get_expression ().size () == 1) {
      return;
    }
    std::string unique_name = m_filename;
    std::replace (unique_name.begin (), unique_name.end (), '/', '_');
    std::replace (unique_name.begin (), unique_name.end (), '.', '_');

    std::string native_name ("nat_" + unique_name + "_" + std::to_string (m_native_num++));
    node->set_build_name (native_name);

    // first pass: generate nat struct
    std::map<std::string,bool> already_handled;
    std::string native_name_struct = native_name + "_struct";
    std::string native_name_obj = native_name + "_obj";

    os << "\n";
    os << "struct " << native_name_struct << " : public NativeExpressionAction {\n";
    os << "\t" << native_name_struct << R"( (ParentProcess *p, const std::string &n, bool string_setter, bool isModel)
      : NativeExpressionAction (p, n, isModel), _string_setter (string_setter)
    {
      set_is_model (isModel);

      // note: finalize_construction will call impl_activate before proper properties are fully built, leading to a crash
      // so delay finalize_construction after building properties
      // finalize_construction (p, n);
    }

    bool _string_setter;
    void impl_deactivate () override {}
    void impl_activate () override;)" << "\n";

    m_expr_in = m_expr_out = 0;
    for (auto n : node->get_output_nodes ()) {
      std::string tn = transform_name(build_fake_name(n, true));
      if(already_handled.count(tn)==0) {
        os << "\tAbstractProperty * " << tn << ";\n";
        already_handled[tn] = true;
      }

      for (auto op : node->get_expression ()) {
        if (op->arg_type () == VAR) {
            std::string tn = transform_name(build_fake_name (op->path_arg_value (), false));
            if(already_handled.count(tn)==0) {
              os << "\tAbstractProperty * " << tn << ";\n";
              already_handled[tn] = true;
            }
          }
      }
    }
    os << "};\n";


    // second pass: generate nat
    m_expr_in = m_expr_out = 0;
    os << "\nvoid\n" << native_name_struct << "::impl_activate ()\n{\n";
    for (auto n : node->get_output_nodes ()) {
      os << "\t" << transform_name (build_fake_name(n, true)) << "->set_value(";
      for (auto op : node->get_expression ()) {
        std::string tn;
        if (op->path_arg_value())
          tn = transform_name (build_fake_name (op->path_arg_value (), false));
        if (op->arg_type () == VAR) {
          switch (op->path_arg_value()->get_cast ()) {
            case BY_PROCESS:
              os << tn;
              break;
            case NO_CAST:
            case BY_REF:
              os << "(*" << tn << ")";
              break;
            case BY_VALUE:
              os << "((AbstractProperty*)" << tn << ")->get_double_value()";
              break;
            default:
            break;
          }
        } else if (tn.size () >= 1
            && tn.at (0) == '\"') {
          os << "std::string (" << build_fake_name (op->path_arg_value (), false) << ")";
        } else {
          os << op->str_arg_value ();
        }
      }
      os << ", " << !node->is_paused () << ");\n";
    }
    indent (os);
    os << "};\n\n";

  }

  void
  CPPBuilder::build_end_define (std::ofstream &os, Node *node)
  {
    indent (os);
    os << "return " << m_parent_list.back ()->get_symbol ("this") << ";\n}\n";
    m_indent--;
    BuildNode* n = m_parent_list.at (m_parent_list.size () - 1);
    m_parent_list.pop_back ();
    if (n)
      delete n;
    n = m_parent_list.at (m_parent_list.size () - 1);
    m_parent_list.pop_back ();
    if (n)
      delete n;
  }

  void
  CPPBuilder::build_instruction (std::ofstream &os, Node *node)
  {
    InstructionNode *n = static_cast<InstructionNode*> (node);
    for (int i = 0; i < n->path_list ().size (); i++) {
      std::string arg = build_find (n->path_list ().at (i), false);
      if (arg.empty ()) {
        print_error_message (error_level::error,
         "unknown component " + n->path_list ().at (i)->get_subpath_list ().at (0)->get_subpath(), 1);
        return;
      }
      indent (os);
      switch (n->type ()) {
        case DUMP:
        os << "if (" << arg << ")" << endl ;
        indent (os); indent (os);
        os << arg << "->dump";
        if (!n->has_argument ()) {
          os << " (0);\n";
          indent (os);
          os << "else" << endl ;
          indent (os); indent (os);
          os << "cout <<  endl << endl << \"warning - dump could not resolve: \" << " << arg <<  " << endl << endl;" << endl;
       }
       else {
        os << " (";
        for (auto arg : n->args ()) {
          build_term_node (os, arg);
        }
        os << ";\n";
      }
      break;
      case XMLSERIALIZE:
        os << "if (" << arg << ")" << endl ;
        indent (os); indent (os);
        os << arg << "->serialize (\"XML\");\n";
        indent (os);
        os << "else" << endl ;
        indent (os); indent (os);
        os << "cout <<  endl << endl << \"warning - XMLSerialize could not resolve: \" << " << arg <<  " << endl << endl;" << endl;
      break;
      case NOTIFY:
      os << arg << "->notify_activate ();\n";
      break;
      case RUN:
      if (n->path_list ().at (i)->get_subpath_list().at(0)->get_subpath().compare("syshook") == 0) {
        if (n->has_argument ()) {
          os << "MainLoop::instance ().set_run_for (";
          for (auto arg : n->args ()) {
            build_term_node (os, arg);
          }
        }
        os << "MainLoop::instance ().activate ();\n";
      } else
      os << arg << "->activate ();\n";
      break;
      case STOP:
      if (n->path_list ().at (i)->get_subpath_list().at(0)->get_subpath().compare("syshook") == 0) {
        os << "MainLoop::instance ().deactivate ();\n";
      } else
      os << arg << "->deactivate ();\n";
      break;
      case DELETE: {
         /* delete first.second */
        std::string new_name ("cpnt_" + std::to_string (m_cpnt_num++));
        os << "auto *" << new_name << " = " << arg << ";\n";
        indent (os);
        os << "if (" << new_name << ") {\n";
        indent (os); indent (os);
        os << new_name << "->deactivate ();\n";
        indent (os); indent (os);
        os << "if (" << new_name << "->get_parent ())\n";
        indent (os); indent (os);
        indent (os);
        os << new_name << "->get_parent ()->remove_child (dynamic_cast<FatChildProcess*>(" << new_name << "));\n";
        indent (os); indent (os);
        os << new_name << "->schedule_delete ();\n";
        indent (os); indent (os);
        os << new_name << " = nullptr;\n";
        indent (os);
        os << "}\n";
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
  CPPBuilder::set_property (std::ofstream &os, Node *node)
  {
    if (!m_in_for)
      indent (os);
    std::string prop_name = node->get_path()->get_subpath_list().at(0)->get_subpath();
    std::string found = m_parent_list.back ()->get_symbol (prop_name);

    // if the symbol is unknown we take it as the definition of a new Process*
    if (found.empty ()) {
      std::string var_name ("cpnt_" + std::to_string (m_cpnt_num++));
      if (m_parent_list.back ()->add_entry (prop_name, var_name) == 1
                && node->duplicate_warning ())
              print_error_message (error_level::warning,
                                   "duplicated name: " + node->name (), 0);
      os << "auto *" << var_name << " = ";
    } else {
      prop_name = build_find (node->get_path(), false);
      if (prop_name.rfind ("cpnt_", 0) == 0) {
        os << "((AbstractProperty*) "<< prop_name << ")->set_value (";
        m_in_set_property = true;
      } else {
        os << prop_name << " = ";
      }
    }
  }

  void
  CPPBuilder::end_set_property (std::ofstream &os, Node *node)
  {
    if (m_in_set_property) {
      os << ", true)";
      m_in_set_property = false;
    }
 //   if (!m_in_for)
   //   os << ";\n";
  }

  void
  CPPBuilder::end_property (std::ofstream &os, Node *node)
  {
    if (node->name ().compare ("Text") == 0)
      os << ")";
    os << ", true);\n";
    m_in_set_text = false;
  }

  void
  CPPBuilder::build_term_node (std::ofstream &os, Node *node)
  {
    TermNode *n = static_cast<TermNode*> (node);
    switch (n->arg_type ()) {
      case SYMBOL:
      case OPERATOR: {
        if (m_in_set_text) {
          os << ")";
        }
        os << n->str_arg_value ();
        if (m_in_set_text) {
          os << "std::string (";
        }
      }
        break;
      case VALUE: {
        if (m_in_set_text) {
          os << "to_string (";
        }
        os << n->str_arg_value ();
        if (m_in_set_text) {
          os << ")";
        }
      }
        break;
      case FUNCTION_CALL:
      case STRING_VALUE:
        os << n->str_arg_value ();
        break;
      case VAR: {
        if (m_in_switch) {
          os << "\"" << n->path_arg_value ()->get_subpath_list().at(0)->get_subpath () << "\"";
          m_in_switch = false;
          return;
        }
        std::string p = build_find (n->path_arg_value (), false);
        os << p;
        break;
      }
      case SMALA_NULL: {
        os << m_null_symbol;
        break;
      }
      case START_LCB_BLOCK: {
        os << "\n";
        indent (os);
        os << "{\n";
        m_indent++;
        break;
      }
      case END_LCB_BLOCK: {
        os << "\n";
        m_indent--;
        indent (os);
        os << "}\n";
        break;
      }
      case END:
        os << ");\n";
        break;
      default:
        return;
    }
  }

  void
  CPPBuilder::get_property (std::ofstream &os, Node *node)
  {
    std::string var_name;
    var_name = ("pr_var_" + std::to_string (m_var_num++));
    if (m_parent_list.back ()->add_entry (node->name (), var_name) == 1)
      print_error_message (error_level::warning,
                           "duplicated name: " + node->name (), 0);
    indent (os);

    if (node->djnn_type ().compare ("doubleToString") == 0) {
      os << "std::string " << var_name
          << " = std::to_string (((DoubleProperty*) ";
    } else if (node->djnn_type ().compare ("intToString") == 0) {
      os << "std::string " << var_name << " = std::to_string (((IntProperty*) ";
    }
    std::string arg = build_find (node->get_path(), false);
    os << arg;
    os << ")->get_value ()";
    if (node->djnn_type ().compare ("doubleToString") == 0
        || node->djnn_type ().compare ("intToString") == 0) {
      os << ")";
    }
    os << ";\n";
  }

  void
  CPPBuilder::alias (std::ofstream &os, Node *node)
  {
    BinaryInstructionNode *n = static_cast<BinaryInstructionNode*> (node);
    std::string new_name ("cpnt_" + std::to_string (m_cpnt_num++));
    indent (os);
    os << "alias (" << m_parent_list.back ()->name () << ", \""
        << n->left_arg ()->get_subpath_list().at(0)->get_subpath() << "\", ";
    std::string arg = build_find (n->right_arg (), false);
    os << "dynamic_cast<FatChildProcess*>(" << arg << "));\n";
    indent (os);
    os << "auto *" << new_name << " = " << m_parent_list.back ()->name ()
        << "->find_child ( \"" << n->left_arg ()->get_subpath_list().at(0)->get_subpath() + "\");\n";
    if (m_parent_list.back ()->add_entry (n->left_arg ()->get_subpath_list().at(0)->get_subpath(), new_name) == 1
        && node->duplicate_warning ())
      print_error_message (error_level::warning,
                           "duplicated name: " + n->left_arg ()->get_subpath_list().at(0)->get_subpath(), 0);
  }

  void
  CPPBuilder::merge (std::ofstream &os, Node *node)
  {
    BinaryInstructionNode *n = static_cast<BinaryInstructionNode*> (node);
    indent (os);
    std::string left = build_find (n->left_arg (), false);
    std::string right = build_find (n->right_arg (), false);
    os << "merge_children (" << left << ", \"\", "
        << right << ", \"\");\n";
  }

  void
  CPPBuilder::remove (std::ofstream &os, Node *node)
  {
    BinaryInstructionNode *n = static_cast<BinaryInstructionNode*> (node);
    indent (os);
    std::string left = build_find (n->left_arg (), false);
    std::string right = build_find (n->right_arg (), false);
    os << left << "->remove_child ( dynamic_cast<FatChildProcess*>(" << right << "));\n";
  }

  void
  CPPBuilder::move (std::ofstream &os, Node *node, const string &c)
  {
    BinaryInstructionNode *n = static_cast<BinaryInstructionNode*> (node);
    indent (os);
    std::string last;
    std::string left = build_find (n->left_arg (), false);
    if (n->right_arg ()) {
      std::string last = build_find (n->right_arg (), false);
      os << left << "->get_parent ()->move_child (dynamic_cast<FatChildProcess*>(" << left << "), "
              << c << ", " << last << ");\n";
    }
    else {
      os << left << "->get_parent ()->move_child (dynamic_cast<FatChildProcess*>(" << left << "), "
        << c << ", nullptr);\n";
    }
  }

  void
  CPPBuilder::add_child (std::ofstream &os, Node *node)
  {
    indent (os);
    std::string new_name ("cpnt_" + std::to_string (m_cpnt_num++));
    if (m_parent_list.back ()->add_entry (node->name (), new_name) == 1) {
      print_error_message (error_level::warning,
                           "duplicated name: " + node->name (), 0);
    }
    m_cur_building_name = new_name;
    os << "auto *" << new_name << " = ";
  }

  void
  CPPBuilder::fetch_add_child (std::ofstream &os, const std::string &parent,
                               const std::string &child, const std::string &name)
  {
    if (parent == m_null_symbol)
      return;
    indent (os);
    os << parent << "->add_child (dynamic_cast<FatChildProcess*>(" << child << "), \"" << name << "\");\n";
  }

  void
  CPPBuilder::add_children_to (std::ofstream &os, Node *node)
  {
    std::string new_name ("cpnt_" + std::to_string (m_cpnt_num++));
    std::string s = build_find (node->get_path(), false);

    if (node->get_path()->get_subpath_list().size ()> 1) {
      indent (os);
      os << "auto *" << new_name << " = " << s << ";\n";
      m_parent_list.push_back (new BuildNode (new_name, m_parent_list.back ()));
      /* FIXME dirty trick to set the parent name of the enclosed nodes*/
      node->set_build_name (new_name);
      indent (os);
      os << "if (" << new_name << " == nullptr)\n";
      indent (os);
      os << "\tcerr <<  endl << endl << \"ERROR - processing addChildrenTo - the component \\\"\" << \"" << node->name () << "\\\"\" << \" is null or do not exist yet\" << endl << endl;\n";
    } else {
      m_parent_list.push_back (new BuildNode (s, m_parent_list.back ()));
      /* FIXME dirty trick to set the parent name of the enclosed nodes*/
      node->set_build_name (s);
    }
  }

  void
  CPPBuilder::build_this_node (std::ofstream &os, Node *node)
  {
    std::string new_name ("cpnt_" + std::to_string (m_cpnt_num++));
    node->set_build_name (new_name);
    if (m_parent_list.back ()->add_entry ("this", new_name) == 1)
      print_error_message (error_level::warning,
                           "duplicated name: " + node->name (), 0);
    indent (os);
    os << "auto* " << new_name << " = new Component  (p, n);\n";

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
  CPPBuilder::build_define_node (std::ofstream &os, Node *node)
  {
    m_parent_list.push_back (new BuildNode ("", m_parent_list.back ()));
    os << "ParentProcess*\n" << node->name () << " (ParentProcess *p, const string &n";
    std::vector< std::pair<ParamType, std::string> > data = node->get_args_data();
    for (int j = 0; j < data.size (); j++) {
      std::pair<ParamType, std::string> arg = data.at (j);
      os << ", ";
      print_type (os, arg.first);
      std::string new_name;
      switch (arg.first) {
        case INT:
          new_name = "i_var_" + std::to_string (m_var_num++);
          break;
        case DOUBLE:
          new_name = "d_var_" + std::to_string (m_var_num++);
          break;
        case STRING:
          new_name = "s_var_" + std::to_string (m_var_num++);
          break;
        case LOCAL_NAME:
        case NAME:
        case PROCESS:
          new_name = "cpnt_" + std::to_string (m_cpnt_num++);
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
  CPPBuilder::build_main_node (std::ofstream &os)
  {

    /* main */
    os << "int\nmain () {\n";
    m_indent = 1;
    int size = m_ast.preamble ().use ().size ();
    bool has_display = false;
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
        os << "init_display ();\n";
        has_display = true;
      }
      if (str == "core") {
        indent (os);
        os << "init_exec_env ();\n";
      }
      /* add cpp init_MODULE corresponding */
      indent (os);
      os << "init_" << str << " ();\n";

    }
  }

  void
  CPPBuilder::build_end_main (std::ofstream &os, Node *node)
  {
    Node* data = (Node*) node->get_user_data ();
    if (data == nullptr)
      return;
    indent (os);
    os << data->build_name () << "->activate ();\n";
    indent (os);
    os << "MainLoop::instance ().activate ();\n";
  }

  void
  CPPBuilder::build_native_action_component (std::ofstream &os, Node *n)
  {
    NativeComponentNode* node = static_cast<NativeComponentNode*> (n);
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
    std::string name =
        node->name ().empty () ? m_null_string : "\"" + node->name () + "\"";
    std::string new_name ("cpnt_" + std::to_string (m_cpnt_num++));
    node->set_build_name (new_name);
    if (!node->name ().empty ()) {
      if (m_parent_list.back ()->add_entry (node->name (), new_name) == 1)
        print_error_message (error_level::warning,
                             "duplicated name: " + node->name (), 0);
    }
    indent (os);
    std::string p_name =
        node->parent () == nullptr ? m_null_symbol : node->parent ()->build_name ();
    os << "auto* " << new_name << " = new " << constructor << " (" << p_name
        << ", " << name << ", " << node->function_name () << ", ";
    if (type == COLLECTION_ACTION) {
      std::string list = build_find (node->path_list(), false);
      os << list << ", ";
    }
    std::string data;
    if (node->path_data() == nullptr)
      data = m_null_symbol;
    else
      data = build_find(node->path_data(), false);
    os << data << ", " << node->is_model () << ");\n ";
  }

  void
  CPPBuilder::build_transition_node (std::ofstream &os, Node *n)
  {
    TransitionNode* ctrl = static_cast<TransitionNode*> (n);
    std::string constructor = get_constructor (ctrl->djnn_type ());
    indent (os);
    os << "new " << constructor << " (" << m_parent_list.back ()->get_symbol (ctrl->parent ()->name())
        << ", " << m_null_string;

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
  CPPBuilder::build_dash_array (std::ofstream &os, DashArrayNode *node)
  {
    std::string name = node->name ().empty () ? m_null_string : "\"" + node->name () + "\"";
    std::string new_name ("cpnt_" + std::to_string (m_cpnt_num++));
    if (node->name ().compare ("_") == 0)
      node->set_name (new_name);
    node->set_build_name (new_name);
    if (m_parent_list.back ()->add_entry (node->name (), new_name) == 1 && node->duplicate_warning ())
      print_error_message (error_level::warning, "duplicated name: " + node->name (), 0);
    indent (os);
    std::string p_name = node->parent () == nullptr ? m_null_symbol : node->parent ()->build_name ();
    os << "DashArray *" << new_name << " =  new DashArray (" << p_name << ", " << name << ");\n";
    int sz = node->get_pattern ().size ();
    if (sz == 0)
      return;
    if (sz == 1) {
      indent (os);
      os << new_name << "->add_sub_pattern (" << node->get_pattern ().at (0) << ", " << node->get_pattern ().at (0) << ");\n";
    } else {
      for (int i = 0; i < sz - 1; i++) {
        indent (os);
        os << new_name << "->add_sub_pattern (" << node->get_pattern ().at (i) << ", " << node->get_pattern ().at (i + 1) << ");\n";
      }
      if (sz % 2 != 0) {
        for (int i = 0; i < sz - 1; i++) {
          indent (os);
          os << new_name << "->add_sub_pattern (" << node->get_pattern ().at (i) << ", " << node->get_pattern ().at (i + 1) << ");\n";
        }
      }
    }
  }

  void
  CPPBuilder::build_smala_native (std::ofstream &os, Node *node)
  {
    SmalaNative *n = static_cast<SmalaNative*> (node);
    std::string src_name = "cpnt_" + std::to_string (m_cpnt_num++);
    std::string data_name = "cpnt_" + std::to_string (m_cpnt_num++);
    m_parent_list.push_back (new BuildNode ("0", m_parent_list.back ()));
    os << "\nstatic void\n" << n->fct () << " (CoreProcess* c) {\n";
    os << "\tauto *" << src_name << " = c->get_activation_source ();\n";
    os << "\tProcess *" << data_name
        << " = (Process *) get_native_user_data (c);\n";
    if (m_parent_list.back ()->add_entry (n->src (), src_name) == 1)
      print_error_message (error_level::warning,
                           "duplicated name: " + n->src (), 0);
    int sz = n->data ()->get_subpath_list ().size();
    std::string user_data_name = sz >= 1 ? n->data ()->get_subpath_list ().at (sz - 1)->get_subpath () : "no_user_data";
    if (m_parent_list.back ()->add_entry (user_data_name, data_name) == 1)
      print_error_message (error_level::warning,
                           "duplicated name: " + user_data_name, 0);
    m_indent = 1;
  }

  void
  CPPBuilder::build_define (const std::string &prefix)
  {
    std::ofstream os (prefix + ".h");
    std::string s = prefix;
    std::replace (s.begin (), s.end (), '/', '_');
    for (int i = 0; i < prefix.length (); i++)
      s.at (i) = std::toupper (s.at (i));
    os << "#pragma once\n#include <string>\n\n";
    for (int i = 0; i < m_ast.define_node_list ().size (); i++) {
      Node *def = m_ast.define_node_list ().at (i);
      os << "djnn::ParentProcess* " << def->name ()
          << " (djnn::ParentProcess*, const std::string &";
      for (int j = 0; j < def->args ().size (); j++) {
        std::pair<ParamType, std::string> arg = def->args ().at (j);
        os << ", ";
        print_type (os, arg.first);
      }
      os << ");\n";
    }
    os.close ();
  }

  void
  CPPBuilder::build_causal_dep (std::ofstream &os, Node* node)
  {
    CausalDependencyNode* n = dynamic_cast<CausalDependencyNode*> (node);
    std::string src = build_find (n->src (), true);
    std::string dst = build_find (n->dst (), true);
    indent (os);
    os << "Graph::instance ().add_edge (" << src << ", " << dst << ");\n";
  }

  void
  CPPBuilder::print_type (std::ofstream &os, ParamType type)
  {
    switch (type) {
      case INT: {
        os << "int";
        break;
      }
      case DOUBLE: {
        os << "double";
        break;
      }
      case STRING: {
        os << "const std::string&";
        break;
      }
      case NAME:
      case PROCESS:
      {
        os << "djnn::CoreProcess*";
        break;
      }
      default:
        break;
    }
  }

  void
  CPPBuilder::print_component_decl (std::ofstream &os, const std::string &name)
  {
    os << "auto* " << name;
  }

  void
  CPPBuilder::print_component_constructor (std::ofstream &os,
                                           const std::string &constructor)
  {
    std::map<std::string, std::string>::iterator it;
    it = m_import_types.find (constructor);
    if (it == m_import_types.end ())
      os << "new " << constructor;
    else
      os << constructor;
  }
} /* namespace Smala */
