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
#include "operator_node.h"
#include "instruction_node.h"
#include "binary_instruction_node.h"
#include "ccall_node.h"
#include "smala_native.h"
#include "ctrl_node.h"
#include "local_node.h"
#include "cpp_type_manager.h"
#include "for_node.h"

#include <locale>
#include <algorithm>

namespace Smala
{

  CPPBuilder::CPPBuilder () :
      Builder ()
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
    m_parent_list.push_back (new BuildNode ("nullptr")); // the first parent is null
    if (!ast.is_main ())
      build_define (prefix);
    m_filename = std::string (prefix) + ".cpp";
    std::ofstream os (prefix + ".cpp");
    os << "#include <iostream>\n";
    os << "#include <string>\n";
    os << "#include \"core/core-dev.h\"\n";
    os << "using namespace std;\nusing namespace djnn;\n\n";

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
    else
      os << "#include \"" << use << "/" << use << ".h\"\n";
  }

  void
  CPPBuilder::build_import (std::ofstream &os, std::string import)
  {
    std::replace (import.begin (), import.end (), '.', '/');
    os << "#include \"" << import << ".h\"\n";
  }

  void
  CPPBuilder::build_while (std::ofstream &os, Node *node)
  {
    indent (os);
    os << "while (";
    for (auto cur: node->get_expression()) {
      build_node (os, cur);
    }
    os << ") {\n";
    m_indent++;
  }

  void
  CPPBuilder::build_for (std::ofstream &os, Node *node)
  {
    ForNode* n = (ForNode*) node;
    indent (os);
    os << "for (";
    //first statement
    Node *first = n->first_st ();
    // if the symbol is unknown we add it as an int
    if (!known_symbol (first->name ())) {
      std::string var_name ("pr_var_" + std::to_string (m_var_num++));
      if (m_parent_list.back ()->add_entry (first->name (), var_name) == 1
          && node->duplicate_warning ())
        print_error_message (error_level::warning,
                             "duplicated name: " + first->name (), 0);
      os << "int ";
    }
    std::pair<std::string, std::string> arg = parse_symbol (first->name ());
    if (arg.first.rfind ("cpnt_", 0) == 0) {
      os << "((AbstractProperty*) ";
      os << arg.first;
      if (arg.second.compare (m_null_string) != 0)
        os << "->find_component (" << arg.second << ")";
      os << ")->set_value (";
      m_in_static_expr = true;
      for (Node *cur : first->get_expression ()) {
        build_node (os, cur);
      }
      m_in_static_expr = false;
      os << ", true);\n";
    } else {
      os << arg.first << " = ";
      m_in_static_expr = true;
      for (Node *cur : first->get_expression ()) {
        build_node (os, cur);
      }
      m_in_static_expr = false;
    }
    os << ";";
    m_in_static_expr = true;

    // second statement
    for (auto cur : n->get_expression ()) {
      build_node (os, cur);
    }
    os << "; ";

    // third statement
    Node *third = n->third_st ();
    arg = parse_symbol (third->name ());
    if (arg.first.rfind ("cpnt_", 0) == 0) {
      os << "((AbstractProperty*) ";
      os << arg.first;
      if (arg.second.compare (m_null_string) != 0)
        os << "->find_component (" << arg.second << ")";
      os << ")->set_value (";
      m_in_static_expr = true;
      for (Node *cur : third->get_expression ()) {
        build_node (os, cur);
      }
      m_in_static_expr = false;
      os << ", true);\n";
    } else {
      os << arg.first << " = ";
      m_in_static_expr = true;
      for (Node *cur : third->get_expression ()) {
        build_node (os, cur);
      }
      m_in_static_expr = false;
    }
    m_in_static_expr = false;
    os << ") {\n";
    m_indent++;
  }

  void
  CPPBuilder::build_control_node (std::ofstream &os, Node *node)
  {
    CtrlNode *ctrl = static_cast<CtrlNode*> (node);
    if (node->djnn_type ().compare ("FSMTransition") == 0) {
      build_transition_node (os, ctrl);
      return;
    }
    std::string constructor = get_constructor (node->djnn_type ());
    //cout << "type = " << node->djnn_type() << " constructor = "  << constructor << endl;
    std::pair<std::string, std::string> src, dst;
    bool is_binding = node->djnn_type ().compare ("Binding") == 0;

    switch (ctrl->in ()->node_type ()) {
      case BINARY_OP: {
        src.first = ctrl->in ()->build_name ();
        src.second = "\"result\"";
        break;
      }
      case UNARY_OP: {
        src.first = ctrl->in ()->build_name ();
        src.second = "\"output\"";
        break;
      }
      case LOCAL_NODE: {
        LocalNode *n = static_cast<LocalNode*> (ctrl->in ());
        src.first = n->root ()->build_name ();
        std::string path = n->path ();
        std::replace (path.begin (), path.end (), '.', '/');
        src.second = "\"" + path + "\"";
        break;
      }
      case LITERAL: {
        std::string new_name ("cpnt_" + std::to_string (m_cpnt_num++));
        indent (os);
        print_start_component (os, new_name, get_constructor ("Double"));
        os << "(" << node->parent ()->build_name () << ", " << m_null_string
            << ", " << ctrl->in ()->name () << ");\n";
        src.first = new_name;
        src.second = m_null_string;
        break;
      }
      case TEXT: {
        std::string new_name ("cpnt_" + std::to_string (m_cpnt_num++));
        indent (os);
        print_start_component (os, new_name, get_constructor ("String"));
        os << "(" << node->parent ()->build_name () << ", " << m_null_string
            << ", " << ctrl->in ()->name () << ");\n";
        src.first = new_name;
        src.second = m_null_string;
        break;
      }
      default:
        if (!ctrl->in ()->build_name ().empty ()) {
          src.first = ctrl->in ()->build_name ();
          src.second = m_null_string;
        } else if (!ctrl->in ()->name ().empty ()) {
          src = parse_symbol (ctrl->in ()->name ());
          std::string prefix = "var_";
          bool _is_var = src.first.substr (0, prefix.size ()) == prefix;
          if (_is_var) {
            std::string new_name ("cpnt_" + std::to_string (m_cpnt_num++));
            indent (os);
            print_start_component (os, new_name, get_constructor ("Double"));
            os << "(" << node->parent ()->build_name () << ", " << m_null_string
                << ", " << src.first << ");\n";
            src.first = new_name;
            src.second = m_null_string;
          }
        } else {
          print_error_message (error_level::warning,
                               "anonymous component in input of control node",
                               1);
          m_error |= 1;
        }
    }

    switch (ctrl->out ()->node_type ()) {
      case LOCAL_NODE: {
        LocalNode *n = static_cast<LocalNode*> (ctrl->out ());
        dst.first = n->root ()->build_name ();
        std::string path = n->path ();
        std::replace (path.begin (), path.end (), '.', '/');
        dst.second = "\"" + path + "\"";
        break;
      }
      default:
        if (!ctrl->out ()->build_name ().empty ()) {
          dst.first = ctrl->out ()->build_name ();
          dst.second = m_null_string;
        } else if (!ctrl->out ()->name ().empty ()) {
          dst = parse_symbol (ctrl->out ()->name ());
        } else {
          print_error_message (error_level::error,
                               "anonymous component in output of control node",
                               1);
        }
    }

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
    os << ", " << src.first << ", " << src.second << ", ";
    if (is_binding)
      os << ctrl->get_in_act () << ", ";
    os << dst.first << ", "
        << dst.second;
    if (is_binding)
      os << ", " << ctrl->get_out_act ();
    if (node->djnn_type ().compare ("Assignment") == 0
        || node->djnn_type ().compare ("PausedAssignment") == 0) {
      os << ", " << node->args ().at (0).second;
    }
    os << ");\n";
  }

  void
  CPPBuilder::build_simple_control_node (std::ofstream &os,
                                         NativeExpressionNode *node)
  {
    std::string p_name =
        node->parent () == nullptr ? "nullptr" : node->parent ()->build_name ();
    ArgNode* arg_node = (ArgNode*) node->get_expression ().at (0);
    std::pair<std::string, std::string> arg;
    if (arg_node->arg_type () != VAR) {
      std::string new_name ("cpnt_" + std::to_string (m_cpnt_num++));
      indent (os);
      if (arg_node->arg_value ().at (0) == '\"') {
        os << "TextProperty *" << new_name << " = new TextProperty (" << p_name
            << ", \"\", " << arg_node->arg_value () << ");\n";
      } else {
        os << "DoubleProperty *" << new_name << " = new DoubleProperty ("
            << p_name << ", \"\", " << arg_node->arg_value () << ");\n";
      }
      arg = std::pair<std::string, std::string> (new_name, "\"\"");
    } else {
      arg = parse_symbol (arg_node->arg_value ());
    }
    if (!node->is_connector () && !node->name ().empty ()) {
      std::string new_name ("cpnt_" + std::to_string (m_cpnt_num++));
      std::pair<std::string, std::string> out_arg = parse_symbol (
          node->get_output_nodes ().at (0));
      os << "Process *" << new_name << " = new Assignment ( " << p_name
          << ", \"\", " << arg.first << ", " << arg.second << ", "
          << out_arg.first << ", " << out_arg.second << ", "
          << node->is_model () << ");\n";
      if (m_parent_list.back ()->add_entry (node->name (), new_name) == 1
          && node->duplicate_warning ())
        print_error_message (error_level::warning,
                             "duplicated name: " + node->name (), 0);
    } else {
      for (auto e : node->get_output_nodes ()) {
        indent (os);
        std::pair<std::string, std::string> out_arg = parse_symbol (e);
        os << "new ";
        if (node->is_paused ())
          os << "Paused";
        if (node->is_connector ())
          os << "Connector (";
        else
          os << "Assignment (";
        os << p_name << ", \"\", " << arg.first << ", " << arg.second << ", "
            << out_arg.first << ", " << out_arg.second;
        if (!node->is_connector ())
          os << ", " << node->is_model ();
        os << ");\n";
      }
    }
  }

  void
  CPPBuilder::build_native_expression_node (std::ofstream &os, Node *n)
  {
    NativeExpressionNode *node = static_cast<NativeExpressionNode*> (n);
    if (node->get_expression ().size () == 1) {
      build_simple_control_node (os, node);
      return;
    }
    std::map<std::string, std::string> sym;
    std::string sym_name ("sym_" + std::to_string (m_sym_num++));
    std::string p_name =
        node->parent () == nullptr ? "nullptr" : node->parent ()->build_name ();
    std::vector<std::string> triggers;
    indent (os);
    os << "std::map<std::string, AbstractProperty*> " << sym_name << ";\n";
    indent (os);
    bool string_setter = false;
    for (auto e : node->get_expression ()) {
      if (((ArgNode*)e)->arg_value ().at (0) == '\"')
        string_setter = true;
    }
    std::string bool_name ("string_setter_" + std::to_string (m_sym_num - 1));
    os << "bool " << bool_name << " = " << string_setter << ";\n";

    for (auto e : node->get_expression ()) {
      if (((ArgNode*)e)->arg_type () == VAR && sym.find (((ArgNode*)e)->arg_value ()) == sym.end ()) {
        std::pair<std::string, std::string> arg = parse_symbol (
            ((ArgNode*)e)->arg_value ());
        if (arg.first.compare (0, 6, "d_var_") == 0
            || arg.first.compare (0, 6, "i_var_") == 0) {
          indent (os);
          std::string new_name ("cpnt_" + std::to_string (m_cpnt_num++));
          os << "DoubleProperty *" << new_name << " = new DoubleProperty ("
              << p_name << ", \"\", " << arg.first << ");\n";
          indent (os);
          os << sym_name << "[\"" << ((ArgNode*)e)->arg_value () << "\"] = " << new_name
              << ";\n";
        } else if (arg.first.compare (0, 6, "s_var_") == 0) {
          indent (os);
          std::string new_name ("cpnt_" + std::to_string (m_cpnt_num++));
          os << "TextProperty *" << new_name << " = new TextProperty ("
              << p_name << ", \"\", " <<arg.first << ");\n";
          indent (os);
          os << sym_name << "[\"" << ((ArgNode*)e)->arg_value () << "\"] = " << new_name
              << ";\n";
          string_setter = true;
        } else {
          indent (os);
          if (arg.second.compare (m_null_string) == 0) {
            os << "if (dynamic_cast<AbstractProperty*>(" << arg.first
                << ") == nullptr) {\n";
            indent (os);
            os << "\tcerr << \"" << ((ArgNode*)e)->arg_value ()
                << "\" << \" is not a property\\n\";\n";
            indent (os);
            os << "\texit(0);\n";
            indent (os);
            os << "} else if (dynamic_cast<TextProperty*>(" << arg.first
                << ") != nullptr) {\n";
            indent (os);
            os << "\t" << bool_name << " = true;\n}\n";
            indent (os);
            os << sym_name << "[\"" << ((ArgNode*)e)->arg_value ()
                << "\"] = dynamic_cast<AbstractProperty*>(" << arg.first
                << ");\n";
            sym[((ArgNode*)e)->arg_value ()] = arg.first;
            triggers.push_back (arg.first);
          } else {
            std::string new_name ("cpnt_" + std::to_string (m_cpnt_num++));
            os << "AbstractProperty* " << new_name
                << " = dynamic_cast<AbstractProperty*> (" << arg.first
                << "->find_component (" << arg.second << "));\n";
            indent (os);
            os << "if (" << new_name << " == nullptr) {\n";
            indent (os);
            os << "\tcerr << \"" << ((ArgNode*)e)->arg_value ()
                << "\" << \" is not a property\\n\";\n";
            indent (os);
            os << "\texit(0);\n";
            indent (os);
            os << "} else if (dynamic_cast<TextProperty*>(" << arg.first
                << "->find_component (" << arg.second << ")) != nullptr) {\n";
            indent (os);
            os << "\t" << bool_name << " = true;\n\t}\n";
            indent (os);
            os << sym_name << "[\"" << ((ArgNode*)e)->arg_value () << "\"] = " << new_name
                << ";\n";
            sym[((ArgNode*)e)->arg_value ()] = new_name;
            triggers.push_back (new_name);
          }
        }
      }
    }
    for (auto e : node->get_output_nodes ()) {
      if (sym.find (e) == sym.end ()) {
        std::pair<std::string, std::string> arg = parse_symbol (e);
        if (arg.first.compare (0, 4, "var_") != 0) {
          indent (os);
          if (arg.second.compare (m_null_string) == 0) {
            os << "if (dynamic_cast<AbstractProperty*>(" << arg.first
                << ") == nullptr) {\n";
            indent (os);
            os << "\tcerr << \"" << e << "\" << \" is not a property\\n\";\n";
            indent (os);
            os << "\texit(0);\n";
            indent (os);
            os << "}\n";
            indent (os);
            os << sym_name << "[\"" << e
                << "\"] = dynamic_cast<AbstractProperty*>(" << arg.first
                << ");\n";
            sym[e] = arg.first;
          } else {
            std::string new_name ("cpnt_" + std::to_string (m_cpnt_num++));
            os << "AbstractProperty* " << new_name
                << " = dynamic_cast<AbstractProperty*> (" << arg.first
                << "->find_component (" << arg.second << "));\n";
            indent (os);
            os << "if (" << new_name << " == nullptr) {\n";
            indent (os);
            os << "\tcerr << \"" << e << "\" << \" is not a property\\n\";\n";
            indent (os);
            os << "\texit(0);\n";
            indent (os);
            os << "}\n";
            indent (os);
            os << sym_name << "[\"" << e << "\"] = " << new_name << ";\n";
            sym[e] = new_name;
          }
        }
      }
    }
    indent (os);
    std::string new_name ("cpnt_" + std::to_string (m_cpnt_num++));
    std::string n_expr_name =
        node->name ().empty () ? m_null_string : node->name ();
    if (string_setter) {
      os << "Process *" << new_name << " = new NativeExpressionAction ("
          << p_name << ", " << n_expr_name << ", " << node->get_build_name ()
          << ", " << sym_name << ", true, " << node->is_model () << ");\n";
    } else {
      os << "Process *" << new_name << " = new NativeExpressionAction ("
          << p_name << ", " << n_expr_name << ", " << node->get_build_name ()
          << ", " << sym_name << ", " << bool_name << ", " << node->is_model ()
          << ");\n";
    }

    if (!node->name ().empty ()) {
      if (m_parent_list.back ()->add_entry (node->name (), new_name) == 1
          && node->duplicate_warning ())
        print_error_message (error_level::warning,
                             "duplicated name: " + node->name (), 0);
    }
    if (node->is_connector ()) {
      for (auto t : triggers) {
        indent (os);
        os << "new Binding (" << p_name << ", \"\", " << t << ", \"\", "
            << new_name << ", \"\");\n";
      }
    }
    for (auto out : node->get_output_nodes ()) {
      std::pair<std::string, std::string> arg = parse_symbol (out);
      indent (os);
      os << "Graph::instance ().add_edge (" << new_name << "," << arg.first;
      if (arg.second.compare (m_null_string) != 0)
        os << "->find_component (" << arg.second << ")";
      os << ");\n";
    }
  }

  void
  CPPBuilder::build_activator (std::ofstream &os, ActivatorNode *n)
  {
    indent (os);
    std::string p_name =
        n->parent () == nullptr ? "nullptr" : n->parent ()->build_name ();
    os << "new Activator (" << p_name << ", \"\", " << n->node ()->build_name ()
        << "->find_component (\"action\"));\n";
  }

  void
  CPPBuilder::build_native_action (std::ofstream &os, Node *n)
  {
    NativeActionNode *node = static_cast<NativeActionNode*> (n);
    os << "void\n";
    os << node->action_name () << "(Process *" << node->param_name () << ")\n";
    const std::string code = node->code ();
    if (code[0] != '{')
      os << "{\n";
    os << code;
    if (code[code.length () - 1] != '}')
      os << "}";
    os << std::endl;
  }

  static bool
  is_sym (const std::string &s)
  {
    return (s.compare ("+") == 0) || (s.compare ("-") == 0)
        || (s.compare ("/") == 0) || (s.compare ("*") == 0)
        || (s.compare (0, 1, "<") == 0) || (s.compare (0, 1, ">") == 0)
        || (s.compare (0, 1, "!") == 0) || (s.compare (0, 1, "&") == 0)
        || (s.compare (0, 1, "|") == 0) || (s.compare (0, 1, "=") == 0);
  }

  static bool
  is_log_sym (const std::string &s)
  {
    return (s.compare (0, 1, "<") == 0) || (s.compare (0, 1, ">") == 0)
        || (s.compare (0, 1, "!") == 0) || (s.compare (0, 1, "&") == 0)
        || (s.compare (0, 1, "|") == 0) || (s.compare (0, 2, "==") == 0);
  }

  void
  CPPBuilder::build_native_expression (std::ofstream &os, Node *n)
  {
    NativeExpressionNode *node = static_cast<NativeExpressionNode*> (n);
    if (node->get_expression ().size () == 1) {
      return;
    }
    bool sym = false;
    bool has_log_sym = false;
    bool has_sym = false;
    bool has_str = false;
    for (auto n_e : node->get_expression ()) {
      if (is_sym (((ArgNode*)n_e)->arg_value ())
          || (((ArgNode*)n_e)->arg_type () == VALUE && ((ArgNode*)n_e)->arg_value ().at (0) != '\"')) {
        if (is_log_sym (((ArgNode*)n_e)->arg_value ()))
          has_log_sym = true;
        sym = true;
      } else if (((ArgNode*)n_e)->arg_value ().compare ("?") == 0) {
        has_log_sym = false;
        sym = false;
      } else if (((ArgNode*)n_e)->arg_value ().compare (":") == 0) {
        if (sym)
          has_sym = true;
      } else if (((ArgNode*)n_e)->arg_value ().at (0) == '\"') {
        has_str = true;
      }
    }
    if (sym)
      has_sym = true;
    std::string native_name ("nat_" + std::to_string (m_native_num++));
    node->set_build_name (native_name);
    os << "\nstatic void\n" << native_name
        << "(std::map<std::string, AbstractProperty*>& sym_table, bool string_setter)\n{\n";
    bool is_string = false;
    for (auto n_e : node->get_expression ()) {
      if (((ArgNode*)n_e)->arg_value ().at (0) == '\"') {
        is_string = true;
        break;
      }
    }

    if (!has_sym) {
      os << "\tif (string_setter) {\n\t\tstd::string result = ";
      int sz = node->get_expression ().size ();
      int i = 1;
      bool in_expr;
      for (auto op : node->get_expression ()) {
        if (((ArgNode*)op)->arg_type () == SYMBOL) {
          os << " " << ((ArgNode*)op)->arg_value () << " ";
          if (is_sym (((ArgNode*)op)->arg_value ())) {
            in_expr = true;
          } else if (((ArgNode*)op)->arg_value ().compare ("?") == 0
              || ((ArgNode*)op)->arg_value ().compare (":") == 0) {
            in_expr = false;
          }
        } else if (((ArgNode*)op)->arg_type () == VAR) {
          if (!in_expr
              && (i == sz
                  || !is_sym (((ArgNode*)node->get_expression ().at (i))->arg_value ()))) {
            os << "((TextProperty*) sym_table.find (\"" << ((ArgNode*)op)->arg_value ()
                << "\")->second)->get_value ()";
          } else {
            os << "sym_table.find (\"" << ((ArgNode*)op)->arg_value ()
                << "\")->second->get_double_value ()";
          }
        } else if (((ArgNode*)op)->arg_value ().at (0) == '\"') {
          os << "std::string (" << ((ArgNode*)op)->arg_value () << ")";
        } else {
          os << ((ArgNode*)op)->arg_value ();
        }
        i++;
      }
      os << ";\n";
      for (auto n : node->get_output_nodes ()) {
        os << "\t\tsym_table.find (\"" << n
            << "\")->second->set_value (result, " << !node->is_paused ()
            << ");\n";
      }
      os << "\t}";
    }
    if (!has_str) {
      if (!has_sym)
        os << "else {\n";
      os << "\t\tdouble result = ";
      for (auto n_e : node->get_expression ()) {
        ArgNode *op = static_cast<ArgNode*> (n_e);
        if (op->arg_type () == SYMBOL) {
          os << " " << op->arg_value () << " ";
        } else if (op->arg_type () == VAR) {
          os << "sym_table.find (\"" << op->arg_value ()
              << "\")->second->get_double_value ()";
        } else if (op->arg_value ().at (0) == '\"') {
          os << "std::string (" << op->arg_value () << ")";
        } else {
          os << op->arg_value ();
        }
      }
      os << ";\n";
      for (auto n : node->get_output_nodes ()) {
        os << "\t\tsym_table.find (\"" << n << "\")->second->set_value (";
        if (has_log_sym)
          os << "(bool)";
        os << "result, " << !node->is_paused () << ");\n";
      }
      if (!has_sym)
        os << "\t}";
    }
    os << "\n}\n";
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
    for (int i = 0; i < n->cpnt_list ().size (); i++) {
      std::pair<std::string, std::string> arg = parse_symbol (
        n->cpnt_list ().at (i));
      std::string cpnt_name =
      arg.second.compare (m_null_string) == 0 ?
      arg.first : arg.first + "->find_component (" + arg.second + ")";
      if (arg.first.empty ()) {
        print_error_message (error_level::error,
         "unknown component " + n->cpnt_list ().at (i), 1);
        return;
      }
      indent (os);
      switch (n->type ()) {
        case DUMP:
        os << cpnt_name << "->dump";
        if (!n->has_argument ())
         os << " (0);\n";
       else {
        os << " (";
        for (auto arg : n->args ()) {
          build_arg_node (os, arg);
        }
        os << ";\n";
      }
      break;
      case NOTIFY:
      os << cpnt_name << "->notify_activation ();\n";
      break;
      case RUN:
      if (n->cpnt_list ().at (i).compare ("syshook") == 0) {
        if (n->has_argument ()) {
          os << "MainLoop::instance ().set_run_for (";
          for (auto arg : n->args ()) {
            build_arg_node (os, arg);
          }
          //os << ");\n";
        }
        indent (os);
        os << "MainLoop::instance ().activation ();\n";
      } else
      os << cpnt_name << "->activation ();\n";
      break;
      case STOP:
      if (cpnt_name.compare ("syshook") == 0) {
        os << "MainLoop::instance ().deactivation ();\n";
      } else
      os << cpnt_name << "->deactivation ();\n";
      break;
      case DELETE:
         /* delete first.second */
      if (arg.second.compare (m_null_string) != 0) {
        std::string new_name ("cpnt_" + std::to_string (m_cpnt_num++));
        os << "Process *" << new_name << " = " << cpnt_name << ";\n";
        indent (os);
        os << "if (" << new_name << ") {\n";
        indent (os); indent (os);
        os << new_name << "->deactivation ();\n";
        indent (os); indent (os);
        os << new_name << "->get_parent ()->remove_child (" << new_name << ");\n";
        indent (os); indent (os);
        os << "delete " << new_name << ";\n";
        indent (os); indent (os);
        os << new_name << " = nullptr;\n";
        indent (os);
        os << "};\n";
      }
        /*  delete first */
      else {
        os << "if (" << arg.first << ") {\n";
        indent (os); indent (os);
        os << arg.first << "->deactivation ();\n";
        indent (os); indent (os);
        os << "if (" << arg.first << "->get_parent ())\n";
        indent (os); indent (os);
        indent (os);
        os << arg.first << "->get_parent ()->remove_child (" << arg.first
        << ");\n";
        indent (os); indent (os);
        os << "delete " << arg.first << ";\n";
        indent (os); indent (os);
        os << arg.first << " = nullptr;\n";
        indent (os);
        os << "};\n";
      }
      break;
      case UNKNOWN:
      print_error_message (error_level::error,
       "unknown instruction " + n->cpnt_list ().at (i), 1);
      break;
    }
  }
}

  void
  CPPBuilder::set_property (std::ofstream &os, Node *node)
  {
    if (node->name ().compare ("set_ref") == 0) {
      set_ref_property (os, node);
      return;
    }
    bool has_string = false;
    for (Node *n : node->get_expression ()) {
      if (((ArgNode*) n)->arg_type () == STRING_VALUE)
        has_string = true;
    }
    indent (os);

    // if the symbol is unknown we add it as a double or as a string
    if (!known_symbol (node->name ())) {
      std::string var_name ("pr_var_" + std::to_string (m_var_num++));
      if (m_parent_list.back ()->add_entry (node->name (), var_name) == 1
                && node->duplicate_warning ())
              print_error_message (error_level::warning,
                                   "duplicated name: " + node->name (), 0);
      if (has_string)
        os << "string ";
      else
        os << "double ";
    }
    std::pair<std::string, std::string> arg = parse_symbol (node->name ());
    if (arg.first.rfind ("cpnt_", 0) == 0) {
      os << "((AbstractProperty*) ";
      os << arg.first;
      if (arg.second.compare (m_null_string) != 0)
        os << "->find_component (" << arg.second << ")";
      os << ")->set_value (";
      if (has_string) {
        os << "string (";
        m_in_set_text = true;
      }
      m_in_static_expr = true;
      for (Node *n: node->get_expression ()) {
        build_node (os, n);
      }
      m_in_static_expr = false;
      if (has_string) {
        os << ")";
        m_in_set_text = false;
      }
      os << ", true);\n";
    } else {
      os << arg.first << " = ";
      if (has_string) {
        os << "string (";
        m_in_set_text = true;
      }
      m_in_static_expr = true;
      for (Node *n : node->get_expression ()) {
        build_node (os, n);
      }
      m_in_static_expr = false;
      if (has_string) {
        os << ")";
        m_in_set_text = false;
      }
      os << ";\n";
    }
  }

  bool
  CPPBuilder::known_symbol (const string& name)
  {
    std::string str;
    std::size_t pos = name.find ('.');
    if (pos == std::string::npos) {
      str = m_parent_list.back ()->get_symbol (name);
      if (str.empty ())
        return false;
    }
    return true;
  }

  void
  CPPBuilder::set_ref_property (std::ofstream &os, Node *node)
  {
    std::string dst = node->args ()[0].second;
    std::string src = node->args ()[1].second;
    indent (os);
    std::pair<std::string, std::string> arg = parse_symbol (dst);
    os << "((RefProperty*) ";
    os << arg.first;
    if (arg.second.compare (m_null_string) != 0)
      os << "->find_component (" << arg.second << ")";
    os << ")->set_value (";
    std::pair<std::string, std::string> arg2 = parse_symbol (src);
    os << arg2.first;
    if (arg2.second.compare (m_null_string) != 0)
      os << "->find_component (" << arg2.second << ")";
    os << ", true);\n";
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
  CPPBuilder::build_arg_node (std::ofstream &os, Node *node)
  {
    ArgNode *n = static_cast<ArgNode*> (node);
    switch (n->arg_type ()) {
      case SYMBOL: {
        if (m_in_set_text) {
          os << ")";
        }
        os << n->arg_value ();
        if (m_in_set_text) {
          os << "string (";
        }
      }
        break;
      case VALUE: {
        if (m_in_set_text) {
          os << "to_string (";
        }
        os << n->arg_value ();
        if (m_in_set_text) {
          os << ")";
        }
      }
        break;
      case STRING_VALUE:
        os << n->arg_value ();
        break;
      case VAR: {
        std::pair<std::string, std::string> p = parse_symbol (n->arg_value ());
        // if the name contains "var_" then this is a simple variable not a djnn property
        // so write it as is and return
        std::size_t found = p.first.find ("var_");
        if (found != std::string::npos) {
          os << p.first;
          return;
        }
        if (m_in_static_expr) {
          if (m_in_set_text)
            os << "((TextProperty*)";
          else
            os << "((AbstractProperty*)";
        }
        if (p.second.compare (m_null_string) == 0)
          os << p.first;
        else
          print_find_component (os, p.first, p.second);
        if (m_in_static_expr) {
          if (m_in_set_text)
            os << ")->get_value ()";
          else
            os << ")->get_double_value ()";
        }
        break;
      }
      case SMALA_NULL: {
        os << m_null_symbol;
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
  CPPBuilder::build_set_string (std::ofstream &os, const std::string &cpnt_name,
                                const std::string &spec,
                                const std::string &value)
  {
    os << "((TextProperty*) " << cpnt_name << "->find_component (" << spec
        << "))->set_value (string (" << value << "), true);\n";
  }

  void
  CPPBuilder::get_property (std::ofstream &os, Node *node)
  {
    std::string var_name;
    if (node->djnn_type ().compare ("GetRef") == 0)
      var_name = string ("cpnt_" + std::to_string (m_cpnt_num++));
    else
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
    } else {
      print_type (os, node->args ().at (0).first);
      os << " " << var_name << " = ";
      os << "((" << node->djnn_type ().substr (3) << "Property*) ";
    }
    std::pair<std::string, std::string> arg = parse_symbol (
        node->args ().at (0).second);
    os << arg.first;
    if (arg.second.compare (m_null_string) != 0)
      os << "->find_component (" << arg.second << ")";
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
        << n->left_arg () << "\", ";
    std::pair<std::string, std::string> arg = parse_symbol (n->right_arg ());
    if (arg.second.compare (m_null_string) == 0)
      os << arg.first << ");\n";
    else
      os << arg.first << "->find_component (" << arg.second << "));\n";
    indent (os);
    os << "Process *" << new_name << " = " << m_parent_list.back ()->name ()
        << "->find_component ( \"" << n->left_arg () + "\");\n";
    if (m_parent_list.back ()->add_entry (n->left_arg (), new_name) == 1
        && node->duplicate_warning ())
      print_error_message (error_level::warning,
                           "duplicated name: " + n->left_arg (), 0);
  }

  void
  CPPBuilder::merge (std::ofstream &os, Node *node)
  {
    BinaryInstructionNode *n = static_cast<BinaryInstructionNode*> (node);
    indent (os);
    std::pair<std::string, std::string> left = parse_symbol (n->left_arg ());
    std::pair<std::string, std::string> right = parse_symbol (n->right_arg ());
    os << "merge_children (" << left.first << ", " << left.second << ", "
        << right.first << ", " << right.second << ");\n";
  }

  void
  CPPBuilder::remove (std::ofstream &os, Node *node)
  {
    BinaryInstructionNode *n = static_cast<BinaryInstructionNode*> (node);
    indent (os);
    std::pair<std::string, std::string> right = parse_symbol (n->left_arg ());
    std::pair<std::string, std::string> left = parse_symbol (n->right_arg ());
    os << left.first << "->remove_child ( " << right.first << ");\n";
  }

  void
  CPPBuilder::move (std::ofstream &os, Node *node, int c)
  {
    BinaryInstructionNode *n = static_cast<BinaryInstructionNode*> (node);
    indent (os);
    string last;
    if (!n->right_arg ().empty())
      last = parse_symbol (n->right_arg ()).first;
    else
      last = "nullptr";
    std::pair<std::string, std::string> left = parse_symbol (n->left_arg ());
    os << left.first << "->get_parent ()->move_child (" << left.first << ", "
        << c << ", " << last << ");\n";
  }

  void
  CPPBuilder::repeat (std::ofstream &os, Node *node)
  {
    std::string new_name ("cpnt_" + std::to_string (m_cpnt_num++));
    if (m_parent_list.back ()->add_entry (node->name (), new_name) == 1)
      print_error_message (error_level::warning,
                           "duplicated name: " + node->name (), 0);

    // TODO we should also parse properly argument expressions

    indent (os);
    os << "Process *" << new_name << " = new List ("
        << m_parent_list.back ()->name () << ", \"" << node->name ()
        << "\");\n";
    indent (os);
    std::string new_var_name ("var_" + std::to_string (m_var_num++));
    os << "int " << new_var_name << ";\n";
    if (m_parent_list.back ()->add_entry (node->args ().at (0).second,
                                          new_var_name) == 1)
      print_error_message (error_level::warning,
                           "duplicated name: " + node->args ().at (0).second,
                           0);

    indent (os);
    string nb_entries =
        node->args ().at (1).first == INT ?
            node->args ().at (1).second :
            m_parent_list.back ()->get_symbol (node->args ().at (1).second);
    os << "for (" << new_var_name << " = 1; " << new_var_name << " <= "
        << nb_entries << "; " << new_var_name << "++) {\n";
    m_indent++;
    std::string inFor ("cpnt_" + std::to_string (m_cpnt_num++));
    indent (os);
    os << "Process *" << inFor << " = new Component (" << new_name
        << ", \"\");\n";
    m_parent_list.push_back (new BuildNode (inFor, m_parent_list.back ()));
    /* FIXME dirty trick to set the parent name of the enclosed nodes*/
    node->set_build_name (inFor);
  }

  void
  CPPBuilder::load_xml (std::ofstream &os, Node *node)
  {
    std::string new_name ("cpnt_" + std::to_string (m_cpnt_num++));
    if (m_parent_list.back ()->add_entry (node->name (), new_name) == 1)
      print_error_message (error_level::warning,
                           "duplicated name: " + node->name (), 0);
    std::pair<ParamType, std::string> arg = node->args ().at (0);
    indent (os);
    os << "Process *" << new_name << " = XML::djnLoadFromXML (";
    if (arg.first == STRING) {
      os << arg.second;
    } else {
      os << m_parent_list.back ()->get_symbol (arg.second);
    }
    os << ");\n";
  }

  void
  CPPBuilder::add_child (std::ofstream &os, Node *node)
  {
    std::pair<std::string, std::string> s = parse_symbol (
        node->args ().at (0).second);
    indent (os);
    if (s.second.compare (m_null_string) == 0) {
      m_parent_list.back ()->add_entry (node->name (), s.first);
      os << m_parent_list.back ()->name () << "->add_child (" << s.first
          << ", \"" << node->name () << "\");\n";
    } else {
      std::string new_name ("cpnt_" + std::to_string (m_cpnt_num++));
      if (m_parent_list.back ()->add_entry (node->name (), new_name) == 1) {
        print_error_message (error_level::warning,
                             "duplicated name: " + node->name (), 0);
      }
      os << "Process *" << new_name << " = " << s.first << "->find_component ("
          << s.second << ");\n";
      indent (os);
      os << m_parent_list.back ()->name () << "->add_child (" << new_name
          << ", \"" << node->name () << "\");\n";
    }
  }

  void
  CPPBuilder::fetch_add_child (std::ofstream &os, std::string &parent,
                               std::string &child, std::string &name)
  {
    indent (os);
    os << parent << "->add_child (" << child << ", \"" << name << "\");\n";
  }

  void
  CPPBuilder::add_children_to (std::ofstream &os, Node *node)
  {
    std::string new_name ("cpnt_" + std::to_string (m_cpnt_num++));
    std::pair<std::string, std::string> s = parse_symbol (node->name ());

    if (s.second.compare (m_null_string) != 0) {
      indent (os);
      os << "Process *" << new_name << " = " << s.first << "->find_component ("
          << s.second << ");\n";
      m_parent_list.push_back (new BuildNode (new_name, m_parent_list.back ()));
      /* FIXME dirty trick to set the parent name of the enclosed nodes*/
      node->set_build_name (new_name);
    } else {
      m_parent_list.push_back (new BuildNode (s.first, m_parent_list.back ()));
      /* FIXME dirty trick to set the parent name of the enclosed nodes*/
      node->set_build_name (s.first);
    }
  }

  void
  CPPBuilder::find (std::ofstream &os, Node *node)
  {
    std::string new_name ("cpnt_" + std::to_string (m_cpnt_num++));
    if (m_parent_list.back ()->add_entry (node->name (), new_name) == 1)
      print_error_message (error_level::warning,
                           "duplicated name: " + node->name (), 0);
    std::pair<ParamType, std::string> arg = node->args ().at (0);
    indent (os);
    os << "Process *" << new_name << " = ";
    if (arg.first == STRING) {
      os << "Process::find_component (nullptr, " << arg.second;
    } else {
      if (node->args ().size () == 2) {
        string root = m_parent_list.back ()->get_symbol (arg.second);
        os << root << "->find_component (" << node->args ().at (1).second;
      } else {
        std::pair<std::string, std::string> p = parse_symbol (arg.second);
        os << p.first << "->find_component (" << p.second;
      }
    }
    os << ");\n";
  }

  void
  CPPBuilder::clone (std::ofstream &os, Node *node)
  {
    std::string new_name ("cpnt_" + std::to_string (m_cpnt_num++));
    if (m_parent_list.back ()->add_entry (node->name (), new_name) == 1)
      print_error_message (error_level::warning,
                           "duplicated name: " + node->name (), 0);
    std::pair<ParamType, std::string> arg = node->args ().at (0);
    indent (os);
    os << "Process *" << new_name << " = ";
    std::pair<std::string, std::string> p = parse_symbol (arg.second);
    if (p.second.compare (m_null_string) == 0)
      os << p.first << "->clone ();\n";
    else
      os << p.first << "->find_component (" << p.second << ")->clone();\n";
  }

  void
  CPPBuilder::build_this_node (std::ofstream &os, Node *node)
  {
    std::string new_name ("cpnt_" + std::to_string (m_cpnt_num++));
    node->set_build_name (new_name);
    if (m_parent_list.back ()->add_entry (node->name (), new_name) == 1)
      print_error_message (error_level::warning,
                           "duplicated name: " + node->name (), 0);
    indent (os);
    os << "Process* " << new_name << " = new Component  (p, n);\n";

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
    os << "Process*\n" << node->name () << " (Process *p, const string &n";
    for (int j = 0; j < node->args ().size (); j++) {
      std::pair<ParamType, std::string> arg = node->args ().at (j);
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
          new_name = "cpnt_" + std::to_string (m_cpnt_num++);
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

    /* init modules from use */
    for (int i = 0; i < size; ++i) {
      std::string str = m_ast.preamble ().use ().at (i);

      /* add cpp init_MODULE corresponding */
      indent (os);
      os << "init_" << str << " ();\n";

    }
  }

  void
  CPPBuilder::build_native_action_component (std::ofstream &os, Node *node)
  {
    std::string constructor = get_constructor (node->djnn_type ());
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
        node->parent () == nullptr ? "nullptr" : node->parent ()->build_name ();
    os << "Process* " << new_name << " = new " << constructor << " (" << p_name
        << ", " << name << ", " << node->args ().at (0).second << ", ";
    std::string data = node->args ().at (1).second;
    if (data.compare ("0") == 0)
      data = "nullptr";
    else {
      std::pair<std::string, std::string> p = parse_symbol (data);
      if (p.second.compare (m_null_string) != 0) {
        data = p.first + "->find_component (" + p.second + ")";
      } else
        data = p.first;
    }
    os << data << ", " << node->args ().at (2).second << ");\n ";
  }

  void
  CPPBuilder::build_transition_node (std::ofstream &os, CtrlNode *ctrl)
  {
    std::string constructor = get_constructor (ctrl->djnn_type ());
    indent (os);
    os << "new " << constructor << " (" << m_parent_list.back ()->name ()
        << ", " << m_null_string;

    std::pair<std::string, std::string> src, dst;
    src = parse_symbol (ctrl->in ()->name ());
    dst = parse_symbol (ctrl->out ()->name ());
    std::string src_str =
        src.second.compare (m_null_string) == 0 ?
            src.first : src.first + "->find_component (" + src.second + ")";
    std::string dst_str =
        dst.second.compare (m_null_string) == 0 ?
            dst.first : dst.first + "->find_component (" + dst.second + ")";
    os << ", " << src_str << ", " << dst_str << ", ";
    std::pair<std::string, std::string> trigger = parse_symbol (
        ctrl->args ().at (0).second);
    os << trigger.first << ", " << trigger.second << ", ";
    if (ctrl->args ().size () == 2) {
      std::pair<std::string, std::string> act = parse_symbol (
          ctrl->args ().at (1).second);
      os << act.first << ", " << act.second << ");\n";
    } else {
      os << "nullptr, \"\");\n";
    }
  }

  void
  CPPBuilder::build_binary_node (std::ofstream &os, Node *node)
  {
    OperatorNode *op = static_cast<OperatorNode*> (node);
    Node *left = op->left ();
    Node *right = op->right ();
    std::string prefix = "var_";
    std::string left_sym = m_parent_list.back ()->get_symbol (left->name ());
    std::string right_sym = m_parent_list.back ()->get_symbol (right->name ());
    bool left_is_var = (left_sym.substr (0, prefix.size ())) == prefix;
    if (left_is_var) {
      left->set_name (left_sym);
    }
    bool right_is_var = right_sym.substr (0, prefix.size ()) == prefix;
    if (right_is_var) {
      right->set_name (right_sym);
    }
    std::string vleft =
        left->node_type () == LITERAL || left_is_var ? left->name () : "0";
    std::string vright =
        right->node_type () == LITERAL || right_is_var ? right->name () : "0";
    std::string constructor = get_constructor (node->djnn_type ());

    std::string new_name = "cpnt_" + std::to_string (m_cpnt_num++);
    m_parent_list.back ()->add_entry (new_name, new_name);
    node->set_build_name (new_name);
    indent (os);
    os << "Process *" << new_name << " = new " << constructor << " ("
        << m_parent_list.back ()->name () << ", \"\", " << vleft << ", "
        << vright << ");\n";
    if (!left_is_var)
      check_and_build_connector (os, left, new_name, "\"left\"");
    if (!right_is_var)
      check_and_build_connector (os, right, new_name, "\"right\"");
    if (node->in_expression ()) {
      indent (os);
      os << "new Activator (" << m_parent_list.back ()->name () << ", \"\", "
          << new_name << ", \"action\");\n";
    }
  }

  void
  CPPBuilder::build_unary_node (std::ofstream &os, Node *node)
  {
    OperatorNode *op = static_cast<OperatorNode*> (node);
    Node *right = op->right ();
    std::string prefix = "var_";
    std::string right_sym = m_parent_list.back ()->get_symbol (right->name ());
    bool right_is_var = right_sym.substr (0, prefix.size ()) == prefix;
    std::string vright =
        right->node_type () == LITERAL || right_is_var ? right->name () : "0";
    std::string constructor = get_constructor (node->djnn_type ());

    std::string new_name = "cpnt_" + std::to_string (m_cpnt_num++);
    m_parent_list.back ()->add_entry (new_name, new_name);
    node->set_build_name (new_name);
    indent (os);
    os << "Process *" << new_name << " = new " << constructor << " ("
        << m_parent_list.back ()->name () << ", \"\", " << vright << ");\n";
    if (!right_is_var)
      check_and_build_connector (os, right, new_name, "\"input\"");
    if (node->in_expression ()) {
      indent (os);
      os << "new Activator (" << m_parent_list.back ()->name () << ", \"\", "
          << new_name << ", \"action\");\n";
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
  CPPBuilder::check_and_build_connector (std::ofstream &os, Node *n,
                                         const std::string &name,
                                         const std::string &side)
  {
    std::pair<std::string, std::string> p;
    switch (n->node_type ()) {
      case PATH: {
        p = parse_symbol (n->name ());
        break;
      }
      case BINARY_OP: {
        p = parse_symbol (n->build_name ());
        p.second = "\"result\"";
        break;
      }
      case UNARY_OP: {
        p = parse_symbol (n->build_name ());
        p.second = "\"output\"";
        break;
      }
      default:
        return;
    }
    indent (os);
    os << "new Connector (" << m_parent_list.back ()->name () << ", \"\", "
        << p.first << ", " << p.second << ", " << name << ", " << side
        << ");\n";
  }

  void
  CPPBuilder::build_smala_native (std::ofstream &os, Node *node)
  {
    SmalaNative *n = static_cast<SmalaNative*> (node);
    std::string src_name = "cpnt_" + std::to_string (m_cpnt_num++);
    std::string data_name = "cpnt_" + std::to_string (m_cpnt_num++);
    m_parent_list.push_back (new BuildNode ("0", m_parent_list.back ()));
    os << "\nstatic void\n" << n->fct () << " (Process* c) {\n";
    os << "\tProcess *" << src_name << " = c->get_activation_source ();\n";
    os << "\tProcess *" << data_name
        << " = (Process *) get_native_user_data (c);\n";
    if (m_parent_list.back ()->add_entry (n->src (), src_name) == 1)
      print_error_message (error_level::warning,
                           "duplicated name: " + n->src (), 0);
    if (m_parent_list.back ()->add_entry (n->data (), data_name) == 1)
      print_error_message (error_level::warning,
                           "duplicated name: " + n->data (), 0);
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
      os << "djnn::Process* " << def->name ()
          << " (djnn::Process*, const std::string &";
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
        os << "const string&";
        break;
      }
      case NAME: {
        os << "djnn::Process*";
        break;
      }
      default:
        break;
    }
  }

  void
  CPPBuilder::print_component_decl (std::ofstream &os, const std::string &name)
  {
    os << "Process* " << name;
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

  std::string
  CPPBuilder::build_find_component (const std::string& first,
                                    const std::string& second)
  {
    return first + "->find_component (" + second + ")";
  }

  void
  CPPBuilder::print_args (std::ofstream &os,
                          std::vector<std::pair<ParamType, std::string> > args,
                          bool is_first)
  {
    int size = args.size ();
    if (size == 0)
      return;
    if (!is_first)
      os << ", ";
    for (int i = 0; i < size; i++) {
      std::string str;
      if (args.at (i).first == NAME) {
        std::pair<std::string, std::string> var = parse_symbol (
            args.at (i).second);
        str =
            var.second.compare ("0") == 0 ?
                var.first : var.first + "->find_component (" + var.second + ")";
      } else {
        str = args.at (i).second;
      }
      os << str;
      if (i < size - 1)
        os << ", ";
    }
  }
} /* namespace Smala */
