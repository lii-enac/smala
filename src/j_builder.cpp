/*  djnn Smala compiler
 *
 *  The copyright holders for the contents of this file are:
 *      Ecole Nationale de l'Aviation Civile, France (2018)
 *  See file "license.terms" for the rights and conditions
 *  defined by copyright holders.
 *
 *
 *  Contributors:
 *      Mathieu Magnaudet <mathieu.magnaudet@enac.fr>
 *      Sébastien Leriche <sebastien.leriche@enac.fr>
 *
 */

#include "j_builder.h"
#include "operator_node.h"
#include "instruction_node.h"
#include "binary_instruction_node.h"
#include "ccall_node.h"
#include "smala_native.h"
#include "ctrl_node.h"
#include "local_node.h"
#include "j_type_manager.h"
#include <cstdlib>
#include <locale>
#include <algorithm>

namespace Smala {

  JBuilder::JBuilder (const std::string& package) :
      Builder (), m_package_name (package) {
    m_type_manager = new JTypeManager ();
    m_null_symbol = "null";
    m_null_string = "\"\"";
  }

  JBuilder::~JBuilder () {
  }

  int
  JBuilder::build (const Ast &ast, const std::string &builddir,
                   const std::string &prefix) {
    m_indent = 0;
    m_cpnt_num = 0;
    m_var_num = 0;
    m_ast = ast;
    m_types.clear ();
    m_parent_list.clear ();
    m_parent_list.push_back (BuildNode ("0")); // the first parent is null

    std::size_t found = prefix.find_last_of ("/"); // we should search the OS dependent file separator
    m_class_name = prefix.substr (found + 1);

    //create package directory from package name
    if (!builddir.empty ()) {
      const int dir_err = system (
          (std::string ("mkdir -p ") + builddir).c_str ());
      if (-1 == dir_err) {
        std::cout << "Unable to create build directory " << builddir << "!\n";
        exit (1);
      }
    }

    m_filename = std::string (builddir) + "/" + m_class_name + ".java";
    std::ofstream os (builddir + "/" + m_class_name + ".java");
    if (!os.good ()) {
      std::cout << "Unable to create java file "
          << builddir + "/" + m_class_name + ".java" << "!\n";
      exit (1);
    }
    os << "package " << m_package_name << ";\n\n";
    build_preamble (os);
    int size = m_ast.preamble ().import ().size ();
    for (int i = 0; i < size; ++i) {
      /* add the import name to the possible types */
      std::string name = m_ast.preamble ().import ().at (i);
      m_types.insert (std::pair<std::string, std::string> (name, name));
    }

    os << "public class " << m_class_name;
    if (!m_ast.is_main ()) {
      os << " extends Component";
    }
    os << "\n{\n";
    m_indent = 1;

    size = m_ast.preamble ().java_code_nodes ().size ();
    for (int i = 0; i < size; ++i) {
      os << m_ast.preamble ().java_code_nodes ().at (i)->code () << std::endl;
    }

    size = m_ast.node_list ().size ();
    for (int i = 0; i < size; ++i) {
      build_node (os, m_ast.node_list ().at (i));
    }
    if (m_ast.is_main ())
      os << "\t}\n";
    os << "}\n";
    os.close ();
    return m_error;
  }

  void
  JBuilder::build_use (std::ofstream &os, std::string use) {
    os << "import net.djnn." << use << ".*;\n";
    if (use.compare ("gui") == 0) {
      os << "import net.djnn.gui.style.*;\n";
      os << "import net.djnn.gui.transformation.*;\n";
    } else if (use.compare ("base") == 0) {
      os << "import net.djnn.base.logic.*;\n";
      os << "import net.djnn.base.numerics.*;\n";
      os << "import net.djnn.base.trigonometry.*;\n";
      os << "import net.djnn.base.text.*;\n";
    }
  }

  void
  JBuilder::build_import (std::ofstream &os, std::string import) {
    os << "import " << m_package_name << "." << import << ";\n";
  }

  void
  JBuilder::build_end_define (std::ofstream &os, Node *node) {
    m_indent--;
    indent (os);
    os << "}\n";
    m_parent_list.pop_back ();
  }

  void
  JBuilder::build_instruction (std::ofstream &os, Node *node) {
    InstructionNode *n = static_cast<InstructionNode*> (node);
    for (int i = 0; i < n->cpnt_list ().size (); i++) {
      std::string cpnt_name = m_parent_list.back ().get_symbol (
          n->cpnt_list ().at (i));
      if (cpnt_name.empty ()) {
        print_error_message (error_level::error,
                             "unknown component " + n->cpnt_list ().at (i), 1);
        return;
      }
      indent (os);
      if (n->name ().compare ("Dump") == 0) {
        os << cpnt_name << ".dumpComponent (0);\n";
      } else if (n->name ().compare ("Run") == 0) {
        if (n->cpnt_list ().at (i).compare ("syshook") == 0) {
          os << "app.startSystemHook ();\n";
        } else
          os << cpnt_name << ".run ();\n";
      } else if (n->name ().compare ("Stop") == 0) {
        if (cpnt_name.compare ("djnSystemHook") == 0) {
          os << "app.stop ();\n";
        } else
          os << cpnt_name << ".stop ();\n";
      } else
        os << cpnt_name << "." << n->name () << "Component ();\n";
    }
  }

  void
  JBuilder::build_set_string (std::ofstream &os, const std::string &cpnt_name,
                              const std::string &spec,
                              const std::string &value) {
    os << get_constructor ("setString") << " (" << cpnt_name << ", " << spec
        << ", " << value << ");\n";
  }

  void
  JBuilder::set_property (std::ofstream &os, Node *node) {
    if (node->name ().compare ("mouseTracking") == 0) {
      indent (os);
      os << "GUI.setMouseTracking (";
      if (node->args ().at (0).second.compare ("0"))
        os << "false);\n";
      else
        os << "true);\n";
      return;
    }
    indent (os);
    std::pair<std::string, std::string> arg = parse_symbol (node->name ());
    switch (node->djnn_type ().at (3))
      {
      case 'B':
        os << "BoolProperty.setBoolPropertyValue";
        break;
      case 'I':
        os << "IntProperty.setIntPropertyValue";
        break;
      case 'D':
        os << "DoubleProperty.setDoublePropertyValue";
        break;
      case 'T':
        os << "TextProperty.setTextPropertyValue";
        break;
      case 'R':
        os << "RefProperty.setRefPropertyValue";
      }
    os << " (" << arg.first << ", " << arg.second << ", ";
    if (node->args ().at (0).first == NAME) {
      std::pair<std::string, std::string> val = parse_symbol (
          node->args ().at (0).second);
      os << val.first << ".findComponent (" << val.second << "));\n";
    } else
      os << node->args ().at (0).second << ");\n";
  }

  void
  JBuilder::get_property (std::ofstream &os, Node *node) {
    std::string var_name ("var_" + std::to_string (m_var_num++));
    if (m_parent_list.back ().add_entry (node->name (), var_name) == 1)
      print_error_message (error_level::warning,
                           "duplicated name: " + node->name (), 0);
    indent (os);
    print_type (os, node->args ().at (0).first);
    std::pair<std::string, std::string> arg = parse_symbol (
        node->args ().at (0).second);
    os << " " << var_name << " = ";

    switch (node->djnn_type ().at (3))
      {
      case 'B':
        os << "BoolProperty.getBoolPropertyValue";
        break;
      case 'I':
        os << "IntProperty.getIntPropertyValue";
        break;
      case 'D':
        os << "DoubleProperty.getDoublePropertyValue";
        break;
      case 'T':
        os << "TextProperty.getTextPropertyValue";
        break;
      case 'R':
        os << "RefProperty.getRefPropertyValue";
      }
    os << " (" << arg.first << ", " << arg.second << ");\n";
  }

  void
  JBuilder::alias (std::ofstream &os, Node *node) {
    BinaryInstructionNode *n = static_cast<BinaryInstructionNode*> (node);
    std::string new_name ("cpnt_" + std::to_string (m_cpnt_num++));
    indent (os);
    os << "new Alias (" << m_parent_list.back ().name () << ", \""
        << n->left_arg () << "\", ";
    std::pair<std::string, std::string> arg = parse_symbol (n->right_arg ());
    if (arg.second.compare ("null") == 0)
      os << arg.first << ");\n";
    else
      os << arg.first << ".findComponent (" << arg.second << "));\n";
    indent (os);
    os << "Component " << new_name << " = "
        << m_parent_list.back ().name () + ".findComponent (\"" + n->left_arg ()
            + "\");\n";
    if (m_parent_list.back ().add_entry (n->left_arg (), new_name) == 1
        && node->duplicate_warning ())
      print_error_message (error_level::warning,
                           "duplicated name: " + n->left_arg (), 0);
  }

  void
  JBuilder::merge (std::ofstream &os, Node *node) {
    BinaryInstructionNode *n = static_cast<BinaryInstructionNode*> (node);
    indent (os);
    std::pair<std::string, std::string> left = parse_symbol (n->left_arg ());
    std::pair<std::string, std::string> right = parse_symbol (n->right_arg ());
    os << "Component.MergeChildren (" << left.first << ", " << left.second
        << ", " << right.first << ", " << right.second << ");\n";
  }

  void
  JBuilder::remove (std::ofstream &os, Node *node) {
    BinaryInstructionNode *n = static_cast<BinaryInstructionNode*> (node);
    indent (os);
    std::pair<std::string, std::string> right = parse_symbol (n->left_arg ());
    std::pair<std::string, std::string> left = parse_symbol (n->right_arg ());
    os << left.first << ".";
    if (left.second.compare ("null") != 0)
      os << "finComponent (left.second).";
    os << "removeChild (" << right.first;
    if (right.second.compare ("null") != 0)
      os << ".findComponent (" << right.second << ")";
    os << ");\n";
  }

  void
  JBuilder::repeat (std::ofstream &os, Node *node) {
    std::string new_name ("cpnt_" + std::to_string (m_cpnt_num++));
    if (m_parent_list.back ().add_entry (node->name (), new_name) == 1)
      print_error_message (error_level::warning,
                           "duplicated name: " + node->name (), 0);

    // TODO we should also parse properly argument expressions

    indent (os);
    os << "Component " << new_name << " = new List ("
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
    os << "Component " << inFor << " = new Component (" << new_name
        << ", null);\n";
    m_parent_list.push_back (
        BuildNode (inFor, m_parent_list.back ().sym_table ()));
    /* FIXME dirty trick to set the parent name of the enclosed nodes*/
    node->set_build_name (inFor);
  }

  void
  JBuilder::load_xml (std::ofstream &os, Node *node) {
    std::string new_name ("cpnt_" + std::to_string (m_cpnt_num++));
    if (m_parent_list.back ().add_entry (node->name (), new_name) == 1)
      print_error_message (error_level::warning,
                           "duplicated name: " + node->name (), 0);
    std::pair<ParamType, std::string> arg = node->args ().at (0);
    indent (os);
    os << "Component " << new_name << " = Component.load (";
    if (arg.first == STRING) {
      os << arg.second;
    } else {
      os << m_parent_list.back ().get_symbol (arg.second);
    }
    os << ");\n";
  }

  void
  JBuilder::add_child (std::ofstream &os, Node *node) {
    std::string new_name ("cpnt_" + std::to_string (m_cpnt_num++));
    if (m_parent_list.back ().add_entry (node->name (), new_name) == 1)
      print_error_message (error_level::warning,
                           "duplicated name: " + node->name (), 0);
    std::pair<std::string, std::string> s = parse_symbol (
        node->args ().at (0).second);
    indent (os);
    os << "Component " << new_name << " = " << s.first << ".findComponent ("
        << s.second << ");\n";
    indent (os);
    os << m_parent_list.back ().name () << ".addChild (" << new_name << ", \""
        << node->name () << "\");\n";
  }

  void
  JBuilder::fetch_add_child (std::ofstream &os, std::string &parent,
                             std::string &child, std::string &name) {
    indent (os);
    os << parent << ".addChild (" << child << ", \"" << name << "\");\n";
  }

  void
  JBuilder::add_children_to (std::ofstream &os, Node *node) {
    std::string new_name ("cpnt_" + std::to_string (m_cpnt_num++));
    indent (os);
    std::pair<std::string, std::string> s = parse_symbol (node->name ());
    os << "Component " << new_name << " = " << s.first << ".findComponent ("
        << s.second << ");\n";
    m_parent_list.push_back (
        BuildNode (new_name, m_parent_list.back ().sym_table ()));
    /* FIXME dirty trick to set the parent name of the enclosed nodes*/
    node->set_build_name (new_name);
  }

  void
  JBuilder::find (std::ofstream &os, Node *node) {
    std::string new_name ("cpnt_" + std::to_string (m_cpnt_num++));
    if (m_parent_list.back ().add_entry (node->name (), new_name) == 1)
      print_error_message (error_level::warning,
                           "duplicated name: " + node->name (), 0);
    std::pair<ParamType, std::string> arg = node->args ().at (0);
    indent (os);
    os << "Component " << new_name << " = Component.findComponent (";
    if (arg.first == STRING) {
      os << "null, " << arg.second;
    } else {
      std::pair<std::string, std::string> p = parse_symbol (arg.second);
      os << p.first << ", " << p.second;
    }
    os << ");\n";
  }

  void
  JBuilder::build_native_action_component (std::ofstream &os, Node *node) {
    std::string name =
        node->name ().empty () ? "null" : "\"" + node->name () + "\"";
    std::string new_name ("cpnt_" + std::to_string (m_cpnt_num++));
    node->set_build_name (new_name);
    std::string cb = node->args ().at (0).second;
    if (!node->name ().empty ()) {
      if (m_parent_list.back ().add_entry (node->name (), new_name) == 1)
        print_error_message (error_level::warning,
                             "duplicated name: " + node->name (), 0);
    }
    indent (os);
    std::string p_name =
        node->parent () == nullptr ? "null" : node->parent ()->build_name ();
    os << "Component " << new_name << " = new NativeAction" << " (" << p_name
        << ", " << name << ", ";
    std::string data = node->args ().at (1).second;
    if (data.compare ("0") == 0)
      data = "null";
    else {
      std::pair<std::string, std::string> p = parse_symbol (data);
      if (p.second.compare ("null") != 0) {
        data = p.first + ".findComponent (" + p.second + ")";
      } else {
        data = p.first;
      }
    }
    os << data << "," << node->args ().at (2).second << ") ";
    std::vector<NativeActionNode*> lst = m_ast.native_list ();
    int size = lst.size ();
    int i = 0;
    NativeActionNode *action = nullptr;
    while (i < size && action == nullptr) {
      std::string act_name = lst.at (i)->action_name ();
      if (act_name.compare (cb) == 0)
        action = lst.at (i);
      i++;
    }
    if (action == nullptr) {
      print_error_message (error_level::error,
                           "native function " + cb + " not found", 1);
      return;
    }
    os << "{\n";
    m_indent++;
    indent (os);
    os << "@Override\n";
    indent (os);
    os << "public void callback (Component e)";
    m_indent++;
    if (action->code ()[0] != '{')
      os << "{\n";
    os << action->code ();
    if (action->code ()[action->code ().length () - 1] != '}')
      os << "}";
    os << std::endl;
    m_indent -= 2;
    indent (os);
    os << "};\n";
  }

  void
  JBuilder::build_this_node (std::ofstream &os, Node *node) {
    node->set_build_name ("this");
    if (m_parent_list.back ().add_entry ("this", "this") == 1)
      print_error_message (error_level::warning,
                           "duplicated name: " + node->name (), 0);

    m_parent_list.push_back (
        BuildNode ("this", m_parent_list.back ().sym_table ()));
  }

  void
  JBuilder::build_define_node (std::ofstream &os, Node *node) {
    os << "\tpublic " << m_class_name << " (Component p, String n";
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
    os << ")\n\t{\n";
    m_indent++;
    indent (os);
    os << "super (p, n);\n";
  }

  void
  JBuilder::build_main_node (std::ofstream &os) {

    /* main */
    os << "\tpublic static void main (String[] args)\n\t{\n";
    os << "\t\tApplication app = Application.getInstance ();\n";
    m_indent++;
    int size = m_ast.preamble ().use ().size ();

    /* init modules from use */
    for (int i = 0; i < size; ++i) {
      std::string str = m_ast.preamble ().use ().at (i);
      std::locale loc;
      for (int j = 0; j < str.length (); j++) {
        str[j] = std::toupper (str[j], loc);
      }
      if (str.compare ("CORE") != 0)
        os << "\t\tapp.initModule (Module." << str << ");\n";
    }
  }

  void
  JBuilder::build_transition_node (std::ofstream &os, CtrlNode *ctrl) {
    std::string constructor = get_constructor (ctrl->djnn_type ());
    indent (os);
    os << "new " << constructor << " (" << m_parent_list.back ().name ()
        << ", null";

    std::pair<std::string, std::string> src, dst;
    src = parse_symbol (ctrl->in ()->name ());
    dst = parse_symbol (ctrl->out ()->name ());
    std::string src_str =
        src.second.compare ("null") == 0 ?
            src.first : src.first + ".findComponent (" + src.second + ")";
    std::string dst_str =
        dst.second.compare ("null") == 0 ?
            dst.first : dst.first + ".findComponent (" + dst.second + ")";
    os << ", " << src_str << ", " << dst_str << ", ";
    std::pair<std::string, std::string> trigger = parse_symbol (
        ctrl->args ().at (0).second);
    os << trigger.first << ", " << trigger.second << ", ";
    if (ctrl->args ().size () == 2) {
      std::pair<std::string, std::string> act = parse_symbol (
          ctrl->args ().at (1).second);
      os << act.first << ", " << act.second << ");\n";
    } else {
      os << "null, null);\n";
    }
  }

  void
  JBuilder::build_binary_node (std::ofstream &os, Node *node) {
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
    os << "Component " << new_name << " = new " << constructor << " ("
        << m_parent_list.back ().name () << ", null, " << vleft << ", "
        << vright << ");\n";
    if (!left_is_var) check_and_build_connector (os, left, new_name, "\"left\"");
    if (!right_is_var) check_and_build_connector (os, right, new_name, "\"right\"");
  }

  void
  JBuilder::build_unary_node (std::ofstream &os, Node *node) {
    OperatorNode *op = static_cast<OperatorNode*> (node);
    Node *right = op->right ();
    std::string prefix = "var_";
    std::string right_sym = m_parent_list.back ().get_symbol (right->name ());
    bool right_is_var = right_sym.substr (0, prefix.size ()) == prefix;
    std::string vright = right->node_type () == LITERAL ? right->name () : "0";
    std::string constructor = get_constructor (node->djnn_type ());

    std::string new_name = "cpnt_" + std::to_string (m_cpnt_num++);
    m_parent_list.back ().add_entry (new_name, new_name);
    node->set_build_name (new_name);
    indent (os);
    os << "Component " << new_name << " = new " << constructor << " ("
        << m_parent_list.back ().name () << ", null, " << vright << ");\n";
    if (!right_is_var) check_and_build_connector (os, right, new_name, "\"input\"");
  }

  void
  JBuilder::check_and_build_connector (std::ofstream &os, Node *n,
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
    os << "new Connector (" << m_parent_list.back ().name () << ", null, "
        << p.first << ", " << p.second << ", " << name << ", " << side
        << ");\n";
  }

  void
  JBuilder::build_smala_native (std::ofstream &os, Node *node) {
    print_error_message (error_level::warning,
                         "Smala native not yet supported in java builder", 0);
  }

  void
  JBuilder::print_type (std::ofstream &os, ParamType type) {
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
          os << "String";
          break;
        }
      case NAME:
        {
          os << "Component";
          break;
        }
      default:
        break;
      }
  }

  void
  JBuilder::print_component_decl (std::ofstream &os, const std::string &name) {
    os << "Component  " << name;
  }

  void
  JBuilder::print_component_constructor (std::ofstream &os,
                                         const std::string &constructor) {
    if (constructor.compare ("Poly.addPointTo") == 0)
      os << constructor;
    else
      os << "new " << constructor;
  }

  std::string
  JBuilder::build_find_component (const std::string& first,
                                  const std::string& second) {
    return first + ".findComponent (" + second + ")";
  }

  void
  JBuilder::print_args (std::ofstream &os,
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
            var.second.compare ("null") == 0 ?
                var.first : var.first + ".findComponent (" + var.second + ")";
      } else {
        str = args.at (i).second;
      }
      os << str;
      if (i < size - 1)
        os << ", ";
    }
  }
} /* namespace Smala */
