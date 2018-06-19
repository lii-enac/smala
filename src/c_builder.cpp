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

#include "c_builder.h"
#include "operator_node.h"
#include "instruction_node.h"
#include "binary_instruction_node.h"
#include "ccall_node.h"
#include "smala_native.h"
#include "ctrl_node.h"
#include "local_node.h"
#include "c_type_manager.h"

#include <locale>
#include <algorithm>

namespace Smala {

  CBuilder::CBuilder () :
      Builder () {
    m_type_manager = new CTypeManager ();
    m_null_symbol = "0";
    m_null_string = "0";
  }

  CBuilder::~CBuilder () {
  }

  int
  CBuilder::build (const Ast &ast, const std::string &builddir,
                   const std::string &prefix) {
    m_indent = 0;
    m_cpnt_num = 0;
    m_var_num = 0;
    m_ast = ast;
    m_types.clear ();
    m_parent_list.clear ();
    m_parent_list.push_back (BuildNode ("0")); // the first parent is null
    if (!ast.is_main ())
      build_define (prefix);
    std::ofstream os (prefix + ".c");

    build_preamble (os);
    int size = m_ast.preamble ().import ().size ();
    for (int i = 0; i < size; ++i) {
      /* add the import name to the possible types */
      std::string name = m_ast.preamble ().import ().at (i);
      m_types.insert (
          std::pair<std::string, std::string> (name, "djnCreate" + name));
    }

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
  CBuilder::build_use (std::ofstream &os, std::string use) {
    os << "#include \"djnn/" << use << "-dev.h\"\n";
  }

  void
  CBuilder::build_import (std::ofstream &os, std::string import) {
    os << "#include \"" << import << ".h\"\n";
  }

  void
  CBuilder::build_native_action (std::ofstream &os, Node *n) {
    NativeActionNode *node = static_cast<NativeActionNode*> (n);
    os << "void\n";
    os << node->action_name () << "(djnComponent *" << node->param_name ()
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
  CBuilder::build_end_define (std::ofstream &os, Node *node) {
    indent (os);
    os << "return " << m_parent_list.back ().get_symbol ("this") << ";\n}\n";
    m_indent--;
    m_parent_list.pop_back ();
    m_parent_list.pop_back ();
  }

  void
  CBuilder::build_instruction (std::ofstream &os, Node *node) {
    InstructionNode *n = static_cast<InstructionNode*> (node);
    for (int i = 0; i < n->cpnt_list ().size (); i++) {
      std::pair<std::string, std::string> arg = parse_symbol (
          n->cpnt_list ().at (i));
      std::string cpnt_name =
          arg.second.compare (m_null_symbol) == 0 ?
              arg.first :
              "djnFindComponent (" + arg.first + ", " + arg.second + ")";
      if (arg.first.empty ()) {
        print_error_message (error_level::error,
                             "unknown component " + n->cpnt_list ().at (i), 1);
        return;
      }
      indent (os);
      if (n->name ().compare ("Dump") == 0) {
        os << "djn_DumpComponentTree (" << cpnt_name << ", 0);\n";
      } else if (n->name ().compare ("Notify") == 0) {
        os << "djn_Notify (" << cpnt_name << ");\n";
      } else {
        os << "djn" << n->name () << "Component (" << cpnt_name << ");\n";
      }
    }
  }

  void
  CBuilder::build_set_string (std::ofstream &os, const std::string &cpnt_name,
                              const std::string &spec,
                              const std::string &value) {
    os << get_constructor ("setString") << " (" << cpnt_name << ", " << spec
        << ", " << value << ");\n";
  }

  void
  CBuilder::set_property (std::ofstream &os, Node *node) {
    if (node->name ().compare ("mouseTracking") == 0) {
      indent (os);
      os << "djn_QtMouseTracking = " << node->args ().at (0).second << ";\n";
      return;
    }
    indent (os);
    os << "djn" << node->djnn_type () << "Property (";
    std::pair<std::string, std::string> arg = parse_symbol (node->name ());
    os << arg.first << ", " << arg.second << ", ";
    if (node->args ().at (0).first == NAME) {
      std::pair<std::string, std::string> val = parse_symbol (
          node->args ().at (0).second);
      os << "djnFindComponent (" << val.first << ", " << val.second << "));\n";
    } else
      os << node->args ().at (0).second << ");\n";
  }

  void
  CBuilder::get_property (std::ofstream &os, Node *node) {
    std::string var_name ("var_" + std::to_string (m_var_num++));
    if (m_parent_list.back ().add_entry (node->name (), var_name) == 1)
      print_error_message (error_level::warning,
                           "duplicated name: " + node->name (), 0);
    indent (os);
    print_type (os, node->args ().at (0).first);
    os << " " << var_name << " = djn" << node->djnn_type ()
        << "PropertyValue (";
    std::pair<std::string, std::string> arg = parse_symbol (
        node->args ().at (0).second);
    os << arg.first << ", " << arg.second << ");\n";
  }

  void
  CBuilder::alias (std::ofstream &os, Node *node) {
    BinaryInstructionNode *n = static_cast<BinaryInstructionNode*> (node);
    std::string new_name ("cpnt_" + std::to_string (m_cpnt_num++));
    indent (os);
    os << "djnCreateAlias (" << m_parent_list.back ().name () << ", \""
        << n->left_arg () << "\", ";
    std::pair<std::string, std::string> arg = parse_symbol (n->right_arg ());
    if (arg.second.empty ())
      os << arg.first << ");\n";
    else
      os << "djnFindComponent (" << arg.first << ", " << arg.second << "));\n";
    indent (os);
    os << "djnComponent *" << new_name
        << " = djnFindComponent (" + m_parent_list.back ().name () + ", \""
            + n->left_arg () + "\");\n";
    if (m_parent_list.back ().add_entry (n->left_arg (), new_name) == 1
        && node->duplicate_warning ())
      print_error_message (error_level::warning,
                           "duplicated name: " + n->left_arg (), 0);
  }

  void
  CBuilder::merge (std::ofstream &os, Node *node) {
    BinaryInstructionNode *n = static_cast<BinaryInstructionNode*> (node);
    indent (os);
    std::pair<std::string, std::string> left = parse_symbol (n->left_arg ());
    std::pair<std::string, std::string> right = parse_symbol (n->right_arg ());
    os << "djnMergeChildren (" << left.first << ", " << left.second << ", "
        << right.first << ", " << right.second << ");\n";
  }

  void
  CBuilder::remove (std::ofstream &os, Node *node) {
    BinaryInstructionNode *n = static_cast<BinaryInstructionNode*> (node);
    indent (os);
    std::pair<std::string, std::string> right = parse_symbol (n->left_arg ());
    std::pair<std::string, std::string> left = parse_symbol (n->right_arg ());
    os << "djnRemoveChild (" << left.first << ",  " << right.first << ");\n";
  }

  void
  CBuilder::clone (std::ofstream &os, Node *node) {
    std::string new_name ("cpnt_" + std::to_string (m_cpnt_num++));
    if (m_parent_list.back ().add_entry (node->name (), new_name) == 1)
      print_error_message (error_level::warning,
                           "duplicated name: " + node->name (), 0);

    // TODO we should also parse properly argument expressions

    indent (os);
    os << "djnComponent *" << new_name << " = djnCreateList ("
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
    os << "djnComponent *" << inFor << " = djnCreateComponent (" << new_name
        << ", 0);\n";
    m_parent_list.push_back (
        BuildNode (inFor, m_parent_list.back ().sym_table ()));
    /* FIXME dirty trick to set the parent name of the enclosed nodes*/
    node->set_build_name (inFor);
  }

  void
  CBuilder::load_xml (std::ofstream &os, Node *node) {
    std::string new_name ("cpnt_" + std::to_string (m_cpnt_num++));
    if (m_parent_list.back ().add_entry (node->name (), new_name) == 1)
      print_error_message (error_level::warning,
                           "duplicated name: " + node->name (), 0);
    std::pair<ParamType, std::string> arg = node->args ().at (0);
    indent (os);
    os << "djnComponent *" << new_name << " = djnLoadFromXML (";
    if (arg.first == STRING) {
      os << arg.second;
    } else {
      os << m_parent_list.back ().get_symbol (arg.second);
    }
    os << ");\n";
  }

  void
  CBuilder::add_child (std::ofstream &os, Node *node) {
    std::string new_name ("cpnt_" + std::to_string (m_cpnt_num++));
    if (m_parent_list.back ().add_entry (node->name (), new_name) == 1)
      print_error_message (error_level::warning,
                           "duplicated name: " + node->name (), 0);
    std::pair<std::string, std::string> s = parse_symbol (
        node->args ().at (0).second);
    indent (os);
    os << "djnComponent *" << new_name << " = djnFindComponent (" << s.first
        << ", " << s.second << ");\n";
    indent (os);
    os << "djnAddChild (" << m_parent_list.back ().name () << ", " << new_name
        << ", \"" << node->name () << "\");\n";
  }

  void
  CBuilder::fetch_add_child (std::ofstream &os, std::string &parent,
                             std::string &child, std::string &name) {
    indent (os);
    os << "djnAddChild (" << parent << ", " << child << ", \"" << name
        << "\");\n";
  }

  void
  CBuilder::add_children_to (std::ofstream &os, Node *node) {
    std::string new_name ("cpnt_" + std::to_string (m_cpnt_num++));
    indent (os);
    std::pair<std::string, std::string> s = parse_symbol (node->name ());
    os << "djnComponent *" << new_name << " = djnFindComponent (" << s.first
        << ", " << s.second << ");\n";
    m_parent_list.push_back (
        BuildNode (new_name, m_parent_list.back ().sym_table ()));
    /* FIXME dirty trick to set the parent name of the enclosed nodes*/
    node->set_build_name (new_name);
  }

  void
  CBuilder::find (std::ofstream &os, Node *node) {
    std::string new_name ("cpnt_" + std::to_string (m_cpnt_num++));
    if (m_parent_list.back ().add_entry (node->name (), new_name) == 1)
      print_error_message (error_level::warning,
                           "duplicated name: " + node->name (), 0);
    std::pair<ParamType, std::string> arg = node->args ().at (0);
    indent (os);
    os << "djnComponent *" << new_name << " = djnFindComponent (";
    if (arg.first == STRING) {
      os << "0, " << arg.second;
    } else {
      std::pair<std::string, std::string> p = parse_symbol (arg.second);
      os << p.first << ", " << p.second;
    }
    os << ");\n";
  }

  void
  CBuilder::build_this_node (std::ofstream &os, Node *node) {
    std::string new_name ("cpnt_" + std::to_string (m_cpnt_num++));
    node->set_build_name (new_name);
    if (m_parent_list.back ().add_entry (node->name (), new_name) == 1)
      print_error_message (error_level::warning,
                           "duplicated name: " + node->name (), 0);
    indent (os);
    os << "djnComponent* " << new_name << " = djnCreateComponent  (p, n);\n";

    m_parent_list.push_back (
        BuildNode (new_name, m_parent_list.back ().sym_table ()));
  }

  void
  CBuilder::build_define_node (std::ofstream &os, Node *node) {
    m_parent_list.push_back (
        BuildNode ("", m_parent_list.back ().sym_table ()));
    os << "djnComponent*\ndjnCreate" << node->name ()
        << " (djnComponent *p, const char *n";
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
  CBuilder::build_main_node (std::ofstream &os) {

    /* main */
    os << "int\nmain () {\n";
    m_indent = 1;
    int size = m_ast.preamble ().use ().size ();

    /* init modules from use */
    for (int i = 0; i < size; ++i) {
      std::string str = m_ast.preamble ().use ().at (i);
      if (str.compare ("gui") == 0)
        str = "GUI";
      else {
        std::locale loc;
        str[0] = std::toupper (str[0], loc);
      }
      indent (os);
      os << "djnInit" << str << " ();\n";
    }

  }

  void
  CBuilder::build_native_action_component (std::ofstream &os, Node *node) {
    std::string constructor = get_constructor (node->djnn_type ());
    std::string name =
        node->name ().empty () ? "0" : "\"" + node->name () + "\"";
    std::string new_name ("cpnt_" + std::to_string (m_cpnt_num++));
    node->set_build_name (new_name);
    if (!node->name ().empty ()) {
      if (m_parent_list.back ().add_entry (node->name (), new_name) == 1)
        print_error_message (error_level::warning,
                             "duplicated name: " + node->name (), 0);
    }
    indent (os);
    std::string p_name =
        node->parent () == nullptr ? "0" : node->parent ()->build_name ();
    os << "djnComponent* " << new_name << " = " << constructor << " (" << p_name
        << ", " << name << ", " << node->args ().at (0).second << ", ";
    std::string data = node->args ().at (1).second;
    if (data.compare ("0") == 0)
      data = "0";
    else {
      std::pair<std::string, std::string> p = parse_symbol (data);
      if (p.second.compare ("0") != 0) {
        data = "djnFindComponent (" + p.first + ", " + p.second + ")";
      } else
        data = p.first;
    }
    os << data << ", " << node->args ().at (2).second << ");\n ";
  }

  void
  CBuilder::build_transition_node (std::ofstream &os, CtrlNode *ctrl) {
    std::string constructor = get_constructor (ctrl->djnn_type ());
    indent (os);
    os << constructor << " (" << m_parent_list.back ().name () << ", 0";

    std::pair<std::string, std::string> src, dst;
    src = parse_symbol (ctrl->in ()->name ());
    dst = parse_symbol (ctrl->out ()->name ());
    std::string src_str =
        src.second.compare ("0") == 0 ?
            src.first :
            "djnFindComponent (" + src.first + ", " + src.second + ")";
    std::string dst_str =
        dst.second.compare ("0") == 0 ?
            dst.first :
            "djnFindComponent (" + dst.first + ", " + dst.second + ")";
    os << ", " << src_str << ", " << dst_str << ", ";
    std::pair<std::string, std::string> trigger = parse_symbol (
        ctrl->args ().at (0).second);
    os << trigger.first << ", " << trigger.second << ", ";
    if (ctrl->args ().size () == 2) {
      std::pair<std::string, std::string> act = parse_symbol (
          ctrl->args ().at (1).second);
      os << act.first << ", " << act.second << ");\n";
    } else {
      os << "0, 0);\n";
    }
  }

  void
  CBuilder::build_binary_node (std::ofstream &os, Node *node)
  {
    OperatorNode *op = static_cast<OperatorNode*> (node);
    Node *left = op->left ();
    Node *right = op->right ();
    std::string prefix = "var_";
    std::string left_sym = m_parent_list.back ().get_symbol (left->name ());
    std::string right_sym = m_parent_list.back ().get_symbol (right->name ());
    bool left_is_var = left_sym.substr (0, prefix.size ()) == prefix;
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
    os << "djnComponent *" << new_name << " = " << constructor << " (" << m_parent_list.back ().name () << ", 0, "
	<< vleft << ", " << vright << ");\n";
    if (!left_is_var) check_and_build_connector (os, left, new_name, "\"left\"");
    if (!right_is_var) check_and_build_connector (os, right, new_name, "\"right\"");
  }

  void
  CBuilder::build_unary_node (std::ofstream &os, Node *node)
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
    os << "djnComponent *" << new_name << " = " << constructor << " (" << m_parent_list.back ().name () << ", 0, "
	<< vright << ");\n";
    if (!right_is_var) check_and_build_connector (os, right, new_name, "\"input\"");
  }

  void
  CBuilder::check_and_build_connector (std::ofstream &os, Node *n,
                                       const std::string &name,
                                       const std::string &side) {
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
    os << "djnCreateConnector (" << m_parent_list.back ().name () << ", 0, "
        << p.first << ", " << p.second << ", " << name << ", " << side
        << ");\n";
  }

  void
  CBuilder::build_smala_native (std::ofstream &os, Node *node) {
    SmalaNative *n = static_cast<SmalaNative*> (node);
    std::string src_name = "cpnt_" + std::to_string (m_cpnt_num++);
    std::string data_name = "cpnt_" + std::to_string (m_cpnt_num++);
    m_parent_list.push_back (
        BuildNode ("0", m_parent_list.back ().sym_table ()));
    os << "\nvoid\n" << n->fct () << " (djnComponent* c) {\n";
    os << "\tdjnComponent *" << src_name << " = djnFindComponent (c, \"^\");\n";
    os << "\tdjnComponent *" << data_name << " = djnGetNativeUserData (c);\n";
    if (m_parent_list.back ().add_entry (n->src (), src_name) == 1)
      print_error_message (error_level::warning,
                           "duplicated name: " + n->src (), 0);
    if (m_parent_list.back ().add_entry (n->data (), data_name) == 1)
      print_error_message (error_level::warning,
                           "duplicated name: " + n->data (), 0);
    m_indent = 1;
  }

  void
  CBuilder::build_define (const std::string &prefix) {
    std::ofstream os (prefix + ".h");
    std::string s = prefix;
    std::replace (s.begin (), s.end (), '/', '_');
    for (int i = 0; i < prefix.length (); i++)
      s.at (i) = std::toupper (s.at (i));
    os << "#ifndef " << s << "_H\n";
    os << "#define " << s << "_H\n\n";
    for (int i = 0; i < m_ast.define_node_list ().size (); i++) {
      Node *def = m_ast.define_node_list ().at (i);
      os << "djnComponent* djnCreate" << def->name ()
          << " (djnComponent*, const char*";
      for (int j = 0; j < def->args ().size (); j++) {
        std::pair<ParamType, std::string> arg = def->args ().at (j);
        os << ", ";
        print_type (os, arg.first);
      }
      os << ");\n";
    }
    os << "\n#endif /*" << s << "_H */\n";
    os.close ();
  }

  void
  CBuilder::print_type (std::ofstream &os, ParamType type) {
    switch (type)
      {
      case INT:
        {
          os << "int";
          break;
        }
      case DOUBLE:
        {
          os << "double";
          break;
        }
      case STRING:
        {
          os << "const char*";
          break;
        }
      case NAME:
        {
          os << "djnComponent*";
          break;
        }
      default:
        break;
      }
  }

  void
  CBuilder::print_component_decl (std::ofstream &os, const std::string &name) {
    os << "djnComponent* " << name;
  }

  void
  CBuilder::print_component_constructor (std::ofstream &os,
                                         const std::string &constructor) {
    os << constructor;
  }

  std::string
  CBuilder::build_find_component (const std::string& first,
                                  const std::string& second) {
    return "djnFindComponent (" + first + ", " + second + ")";
  }

  void
  CBuilder::print_args (std::ofstream &os,
                        std::vector<std::pair<ParamType, std::string> > args,
                        bool is_first) {
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
                var.first :
                "djnFindComponent (" + var.first + ", " + var.second + ")";
      } else {
        str = args.at (i).second;
      }
      os << str;
      if (i < size - 1)
        os << ", ";
    }
  }
} /* namespace Smala */
