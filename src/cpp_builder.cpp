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
#include "operator_node.h"
#include "instruction_node.h"
#include "binary_instruction_node.h"
#include "ccall_node.h"
#include "smala_native.h"
#include "ctrl_node.h"
#include "local_node.h"
#include "cpp_type_manager.h"

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
    m_parent_list.push_back (BuildNode ("nullptr")); // the first parent is null
    if (!ast.is_main ())
      build_define (prefix);
    m_filename = std::string (prefix) + ".cpp";
    std::ofstream os (prefix + ".cpp");
    os << "#include <iostream>\n";
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
    os << "#include \"" << import << ".h\"\n";
  }

  void
  CPPBuilder::build_activator (std::ofstream &os, ActivatorNode *n)
  {
    indent (os);
    std::string p_name =
    n->parent () == nullptr ? "nullptr" : n->parent ()->build_name ();
    os << "new Activator (" << p_name << ", \"\", " << n->node()->build_name () << "->find_component (\"action\"));\n";
  }

  void
  CPPBuilder::build_native_action (std::ofstream &os, Node *n)
  {
    NativeActionNode *node = static_cast<NativeActionNode*> (n);
    os << "void\n";
    os << node->action_name () << "(Process *" << node->param_name ()
        << ")\n";
    const std::string code = node->code ();
    if (code[0] != '{')
      os << "{\n";
    os << code;
    if (code[code.length () - 1] != '}')
      os << "}";
    os << std::endl;
  }

  void
  CPPBuilder::build_end_define (std::ofstream &os, Node *node)
  {
    indent (os);
    os << "return " << m_parent_list.back ().get_symbol ("this") << ";\n}\n";
    m_indent--;
    m_parent_list.pop_back ();
    m_parent_list.pop_back ();
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
      if (n->name ().compare ("Dump") == 0) {
        os << cpnt_name << "->dump (0);\n";
      } else if (n->name ().compare ("Notify") == 0) {
        os << cpnt_name << "->notify_activation ();\n";
      } else if (n->name ().compare ("Run") == 0) {
        if (n->cpnt_list ().at (i).compare ("syshook") == 0) {
          os << "MainLoop::instance ().activation ();\n";
        } else
          os << cpnt_name << "->activation ();\n";
      } else if (n->name ().compare ("Stop") == 0) {
        if (cpnt_name.compare ("syshook") == 0) {
          os << "MainLoop::instance ().deactivation ();\n";
        } else
          os << cpnt_name << "->deactivation ();\n";
      } else if (n->name ().compare ("Delete") == 0) {

        /* delete first.second */
        if (arg.second.compare (m_null_string) != 0) {
          std::string new_name ("cpnt_" + std::to_string (m_cpnt_num++));
          os << "Process *" << new_name << " = " << cpnt_name << ";\n";
          indent (os);
          os << new_name << "->deactivation ();\n";
          indent (os);
          os << arg.first << "->remove_child (" << new_name << ");\n";
          indent (os);
          os << "delete " << new_name << ";\n";
        }
        /*  delete first */
        else {
          os << arg.first << "->deactivation ();\n";
          indent (os);
          os << "if (" << arg.first << "->get_parent ())\n";
          indent (os); indent (os);
          os << arg.first << "->get_parent ()->remove_child (" << arg.first << ");\n";
          indent (os);
          os << "delete " << arg.first << ";\n";
        }
      }
    }
  }

  void
  CPPBuilder::set_property (std::ofstream &os, Node *node)
  {
    if (node->name ().compare ("mouseTracking") == 0) {
      indent (os);
      os << "mouse_tracking = " << node->args ().at (0).second << ";\n";
      return;
    }
    if (node->name ().compare ("fullScreen") == 0) {
      indent (os);
      os << "full_screen = " << node->args ().at (0).second << ";\n";
      return;
    }
    indent (os);
    os << "((" << node->djnn_type ().substr (3) << "Property*) ";
    std::pair<std::string, std::string> arg = parse_symbol (node->name ());
    os << arg.first << "->find_component ( " << arg.second << "))->set_value (";
    if (node->djnn_type ().substr (3).compare ("Text") == 0) {
      os << "string (";
    }
    if (node->args ().at (0).first == NAME) {
      std::pair<std::string, std::string> val = parse_symbol (
          node->args ().at (0).second);
      os << val.first << "->find_component (" << val.second << ")";
    } else
      os << node->args ().at (0).second;
    if (node->djnn_type ().substr (3).compare ("Text") == 0) {
      os << ")";
    }
    os << ", true);\n";
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
    std::string var_name ("pr_var_" + std::to_string (m_var_num++));
    if (m_parent_list.back ().add_entry (node->name (), var_name) == 1)
      print_error_message (error_level::warning,
                           "duplicated name: " + node->name (), 0);
    indent (os);
    print_type (os, node->args ().at (0).first);
    os << " " << var_name << " = ((" << node->djnn_type ().substr (3)
        << "Property*) ";
    std::pair<std::string, std::string> arg = parse_symbol (
        node->args ().at (0).second);
    os << arg.first << "->find_component (" << arg.second
        << "))->get_value ();\n";
  }

  void
  CPPBuilder::alias (std::ofstream &os, Node *node)
  {
    BinaryInstructionNode *n = static_cast<BinaryInstructionNode*> (node);
    std::string new_name ("cpnt_" + std::to_string (m_cpnt_num++));
    indent (os);
    os << "alias (" << m_parent_list.back ().name () << ", \"" << n->left_arg ()
        << "\", ";
    std::pair<std::string, std::string> arg = parse_symbol (n->right_arg ());
    if (arg.second.compare (m_null_string) == 0)
      os << arg.first << ");\n";
    else
      os << arg.first << "->find_component (" << arg.second << "));\n";
    indent (os);
    os << "Process *" << new_name << " = "
        << m_parent_list.back ().name () << "->find_component ( \""
        << n->left_arg () + "\");\n";
    if (m_parent_list.back ().add_entry (n->left_arg (), new_name) == 1
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
  CPPBuilder::clone (std::ofstream &os, Node *node)
  {
    std::string new_name ("cpnt_" + std::to_string (m_cpnt_num++));
    if (m_parent_list.back ().add_entry (node->name (), new_name) == 1)
      print_error_message (error_level::warning,
                           "duplicated name: " + node->name (), 0);

    // TODO we should also parse properly argument expressions

    indent (os);
    os << "Process *" << new_name << " = new List ("
        << m_parent_list.back ().name () << ", \"" << node->name () << "\");\n";
    indent (os);
    std::string new_var_name ("var_" + std::to_string (m_var_num++));
    os << "int " << new_var_name << ";\n";
    if (m_parent_list.back ().add_entry (node->args ().at (0).second,
                                         new_var_name) == 1)
      print_error_message (error_level::warning,
                           "duplicated name: " + node->args ().at (0).second,
                           0);

    indent (os);
    string nb_entries =
        node->args ().at (1).first == INT ?
            node->args ().at (1).second :
            m_parent_list.back ().get_symbol (node->args ().at (1).second);
    os << "for (" << new_var_name << " = 1; " << new_var_name << " <= "
        << nb_entries << "; " << new_var_name << "++) {\n";
    m_indent++;
    std::string inFor ("cpnt_" + std::to_string (m_cpnt_num++));
    indent (os);
    os << "Process *" << inFor << " = new Component (" << new_name
        << ", \"\");\n";
    m_parent_list.push_back (
        BuildNode (inFor, m_parent_list.back ().sym_table ()));
    /* FIXME dirty trick to set the parent name of the enclosed nodes*/
    node->set_build_name (inFor);
  }

  void
  CPPBuilder::load_xml (std::ofstream &os, Node *node)
  {
    std::string new_name ("cpnt_" + std::to_string (m_cpnt_num++));
    if (m_parent_list.back ().add_entry (node->name (), new_name) == 1)
      print_error_message (error_level::warning,
                           "duplicated name: " + node->name (), 0);
    std::pair<ParamType, std::string> arg = node->args ().at (0);
    indent (os);
    os << "Process *" << new_name << " = XML::djnLoadFromXML (";
    if (arg.first == STRING) {
      os << arg.second;
    } else {
      os << m_parent_list.back ().get_symbol (arg.second);
    }
    os << ");\n";
  }

  void
  CPPBuilder::add_child (std::ofstream &os, Node *node)
  {
    std::string new_name ("cpnt_" + std::to_string (m_cpnt_num++));
    if (m_parent_list.back ().add_entry (node->name (), new_name) == 1)
      print_error_message (error_level::warning,
                           "duplicated name: " + node->name (), 0);
    std::pair<std::string, std::string> s = parse_symbol (
        node->args ().at (0).second);
    indent (os);
    os << "Process *" << new_name << " = " << s.first
        << "->find_component (" << s.second << ");\n";
    indent (os);
    os << m_parent_list.back ().name () << "->add_child (" << new_name << ", \""
        << node->name () << "\");\n";
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
      os << "Process *" << new_name << " = " << s.first
      << "->find_component (" << s.second << ");\n";
      m_parent_list.push_back (
        BuildNode (new_name, m_parent_list.back ().sym_table ()));
    /* FIXME dirty trick to set the parent name of the enclosed nodes*/
      node->set_build_name (new_name);
    }
    else {
      m_parent_list.push_back (
        BuildNode (s.first, m_parent_list.back ().sym_table ()));
    /* FIXME dirty trick to set the parent name of the enclosed nodes*/
      node->set_build_name (s.first);
    }
  }

  void
  CPPBuilder::find (std::ofstream &os, Node *node)
  {
    std::string new_name ("cpnt_" + std::to_string (m_cpnt_num++));
    if (m_parent_list.back ().add_entry (node->name (), new_name) == 1)
      print_error_message (error_level::warning,
                           "duplicated name: " + node->name (), 0);
    std::pair<ParamType, std::string> arg = node->args ().at (0);
    indent (os);
    os << "Process *" << new_name << " = ";
    if (arg.first == STRING) {
      os << "Process::find_component (nullptr, " << arg.second;
    } else {
      if (node->args ().size() == 2) {
        string root = m_parent_list.back ().get_symbol (arg.second);
        os << root << "->find_component (" << node->args ().at (1).second;
      } else {
        std::pair<std::string, std::string> p = parse_symbol (arg.second);
        os << p.first << "->find_component (" << p.second;
      }
    }
    os << ");\n";
  }

  void
  CPPBuilder::build_this_node (std::ofstream &os, Node *node)
  {
    std::string new_name ("cpnt_" + std::to_string (m_cpnt_num++));
    node->set_build_name (new_name);
    if (m_parent_list.back ().add_entry (node->name (), new_name) == 1)
      print_error_message (error_level::warning,
                           "duplicated name: " + node->name (), 0);
    indent (os);
    os << "Process* " << new_name << " = new Component  (p, n);\n";

    m_parent_list.push_back (
        BuildNode (new_name, m_parent_list.back ().sym_table ()));
  }

  void
  CPPBuilder::build_define_node (std::ofstream &os, Node *node)
  {
    m_parent_list.push_back (
        BuildNode ("", m_parent_list.back ().sym_table ()));
    os << "Process*\n" << node->name ()
        << " (Process *p, const string &n";
    for (int j = 0; j < node->args ().size (); j++) {
      std::pair<ParamType, std::string> arg = node->args ().at (j);
      os << ", ";
      print_type (os, arg.first);
      std::string new_name;
      if (arg.first != NAME)
        new_name = "var_" + std::to_string (m_var_num++);
      else
        new_name = "cpnt_" + std::to_string (m_cpnt_num++);
      os << " " << new_name;
      if (m_parent_list.back ().add_entry (arg.second, new_name) == 1)
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
      if (m_parent_list.back ().add_entry (node->name (), new_name) == 1)
        print_error_message (error_level::warning,
                             "duplicated name: " + node->name (), 0);
    }
    indent (os);
    std::string p_name =
        node->parent () == nullptr ? "nullptr" : node->parent ()->build_name ();
    os << "Process* " << new_name << " = new " << constructor << " ("
        << p_name << ", " << name << ", " << node->args ().at (0).second
        << ", ";
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
    os << "new " << constructor << " (" << m_parent_list.back ().name () << ", "
        << m_null_string;

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
    std::string left_sym = m_parent_list.back ().get_symbol (left->name ());
    std::string right_sym = m_parent_list.back ().get_symbol (right->name ());
    bool left_is_var = (left_sym.substr (0, prefix.size ())) == prefix;
    if (left_is_var) {
      left->set_name (left_sym);
    }
    bool right_is_var = right_sym.substr (0, prefix.size ()) == prefix;
    if (right_is_var) {
      right->set_name (right_sym);
    }
    std::string vleft = left->node_type () == LITERAL || left_is_var ? left->name () : "0";
    std::string vright = right->node_type () == LITERAL || right_is_var ? right->name () : "0";
    std::string constructor = get_constructor (node->djnn_type ());

    std::string new_name = "cpnt_" + std::to_string (m_cpnt_num++);
    m_parent_list.back ().add_entry (new_name, new_name);
    node->set_build_name (new_name);
    indent (os);
    os << "Process *" << new_name << " = new " << constructor << " (" << m_parent_list.back ().name () << ", \"\", "
        << vleft << ", " << vright << ");\n";
    if (!left_is_var)
      check_and_build_connector (os, left, new_name, "\"left\"");
    if (!right_is_var)
      check_and_build_connector (os, right, new_name, "\"right\"");
    if (node->in_expression ()) {
      indent (os);
      os << "new Activator (" << m_parent_list.back ().name () << ", \"\", " << new_name << ", \"action\");\n";
    }
  }

  void
  CPPBuilder::build_unary_node (std::ofstream &os, Node *node)
  {
    OperatorNode *op = static_cast<OperatorNode*> (node);
    Node *right = op->right ();
    std::string prefix = "var_";
    std::string right_sym = m_parent_list.back ().get_symbol (right->name ());
    bool right_is_var = right_sym.substr (0, prefix.size ()) == prefix;
    std::string vright = right->node_type () == LITERAL || right_is_var ? right->name () : "0";
    std::string constructor = get_constructor (node->djnn_type ());

    std::string new_name = "cpnt_" + std::to_string (m_cpnt_num++);
    m_parent_list.back ().add_entry (new_name, new_name);
    node->set_build_name (new_name);
    indent (os);
    os << "Process *" << new_name << " = new " << constructor << " (" << m_parent_list.back ().name () << ", \"\", "
        << vright << ");\n";
    if (!right_is_var)
      check_and_build_connector (os, right, new_name, "\"input\"");
    if (node->in_expression ()) {
      indent (os);
      os << "new Activator (" << m_parent_list.back ().name () << ", \"\", " << new_name << ", \"action\");\n";
    }
  }

  void
  CPPBuilder::check_and_build_connector (std::ofstream &os, Node *n, const std::string &name, const std::string &side)
  {
    std::pair<std::string, std::string> p;
    switch (n->node_type ())
      {
      case PATH:
        {
          p = parse_symbol (n->name ());
          break;
        }
      case BINARY_OP:
        {
          p = parse_symbol (n->build_name ());
          p.second = "\"result\"";
          break;
        }
      case UNARY_OP:
        {
          p = parse_symbol (n->build_name ());
          p.second = "\"output\"";
          break;
        }
      default:
        return;
      }
    indent (os);
    os << "new Connector (" << m_parent_list.back ().name () << ", \"\", " << p.first << ", " << p.second << ", "
        << name << ", " << side << ");\n";
  }

  void
  CPPBuilder::build_smala_native (std::ofstream &os, Node *node)
  {
    SmalaNative *n = static_cast<SmalaNative*> (node);
    std::string src_name = "cpnt_" + std::to_string (m_cpnt_num++);
    std::string data_name = "cpnt_" + std::to_string (m_cpnt_num++);
    m_parent_list.push_back (
        BuildNode ("0", m_parent_list.back ().sym_table ()));
    os << "\nstatic void\n" << n->fct () << " (Process* c) {\n";
    os << "\tProcess *" << src_name
        << " = c->get_activation_source ();\n";
    os << "\tProcess *" << data_name
        << " = (Process *) get_native_user_data (c);\n";
    if (m_parent_list.back ().add_entry (n->src (), src_name) == 1)
      print_error_message (error_level::warning,
                           "duplicated name: " + n->src (), 0);
    if (m_parent_list.back ().add_entry (n->data (), data_name) == 1)
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
