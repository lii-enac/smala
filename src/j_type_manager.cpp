/*
 *  djnn Smala compiler
 *
 *  The copyright holders for the contents of this file are:
 *    Ecole Nationale de l'Aviation Civile, France (2017)
 *  See file "license.terms" for the rights and conditions
 *  defined by copyright holders.
 *
 *
 *  Contributors:
 *    Mathieu Magnaudet <mathieu.magnaudet@enac.fr>
 *    Stephane Conversy <stephane.conversy@enac.fr>
 *
 */

#include "j_type_manager.h"

#include <iostream>

namespace Smala {


  JTypeManager::JTypeManager () : TypeManager ()
  {
    type_entry coreGS[] = {
      //{"ComponentSetAdded", "djnComponentSetAdded"},
      //{"ComponentSetRemoved", "djnComponentSetRemoved"},
      {"syshook", "app.startSystemHook ()"},
      {"mainloop", "app.startMainLoop ()"},
      {"0", "0"}
    };
    coreGlobalSymbols = coreGS;

    type_entry displayGS[] = {
      {"Displays", "Display.getDysplays ()"},
      {"0", "0"}
    };
    displayGlobalSymbols = displayGS;

    type_entry inputGS[] = {
      {"InputDevices", "Input.getInputDevices ()"},
      {"Mice", "Input.getMice ()"},
      {"TouchPanels", "Input.getTouchPanels ()"},
      {"Styluses", "Input.getStyluses ()"},
      {"RawPointers", "Input.RawPointers ()"},
      {"Joysticks", "Input.Joysticks ()"},
      {"RotaryKnobs", "Input.RotaryKnobs ()"},
      {"Absolute3DDevices", "Input.Absolute3DDevices ()"},
      {"MakeyMakeys", "Input.MakeyMakeys ()"},
      {"Keyboards", "Input.Keyboards ()"},
      {"0", "0"}
    };
    inputGlobalSymbols = inputGS;

    type_entry phidgetsGS[] = {
      {"InterfaceKits", "Phidgets.getInterfaceKits ()"},
      {"0", "0"}
    };
    phidgetsGlobalSymbols = phidgetsGS;

    type_entry powerGS[] = {
      {"Batteries", "Power.getBatteries ()"},
      {"0", "0"}
    };
    powerGlobalSymbols = powerGS;

    type_entry guiGS[] = {
      {"mouseTracking", "GUI.setMouseTracking"},
      {"0", "0"}
    };
    guiGlobalSymbols = guiGS;

    type_entry soundGS[] = {
      {"PortAudio", "Sound.getPortAudio ()"},
      {"0", "0"}
    };
    soundGlobalSymbols = soundGS;

    type_entry animation_t[] = {
      {"SlowInSlowOutInterpolator","SlowInSlowOutInterpolator"},
      {"0", "0"}
    };
    animation_types = animation_t;

    type_entry base_t[] = {
      {"Clock", "Clock"},
      {"Exit", "Exit"},
      {"FSM", "FSM"},
      {"FSMState", "FSMState"},
      {"FSMTransition", "FSMTransition"},
      {"Switch", "Switch"},
      {"Or", "Or"},
      {"Xor", "Xor"},
      {"And", "And"},
      {"Not", "Not"},
      {"Incr", "Incr"},
      {"Pow", "Pow"},
      {"Adder", "Adder"},
      {"Subtractor", "Subtractor"},
      {"Multiplier", "Multiplier"},
      {"Divider", "Divider"},
      {"SignInverter", "SignInverter"},
      {"EqualityComparator", "EqualityComparator"},
      {"AscendingComparator", "AscendingComparator"},
      {"StrictAscendingComparator", "StrictAscendingComparator"},
      {"AdderAccumulator", "AdderAccumulator"},
      {"Normalizer", "Normalizer"},
      {"HermiteCurve", "HermiteCurve"},
      {"Sine", "Sine"},
      {"Cosine", "Cosine"},
      {"Tangent", "Tangent"},
      {"RandomGenerator", "RandomGenerator"},
      {"Formula", "Formula"},
      {"IntToDouble", "IntToDouble"},
      {"TextCatenator", "TextCatenator"},
      {"IntFormatter", "IntFormatter"},
      {"DoubleFormatter", "DoubleFormatter"},
      {"TextPrinter", "TextPrinter"},
      {"TextComparator", "TextComparator"},
      {"LogPrinter", "LogPrinter"},
      {"RegexpFilter", "RegexpFilter"},
      {"PN", "PN"},
      {"PNPlace", "PNPlace"},
      {"PNTransition", "PNTransition"},
      {"PNInputTransition", "PNInputTransition"},
      {"PNArc", "PNArc"},
      {"Noop", "Noop"},
      {"Counter", "Counter"},
      {"JsonSerializeComponent", "djnJsonSerializeComponent"},
      {"0", "0"}
    };
    base_types = base_t;

    type_entry comms_t[] = {
      {"IvyAccess", "IvyAccess"},
      {"IvyBus", "IvyBus"},
      {"IvyExporter", "IvyExporter"},
      {"IvyImporter", "IvyImporter"},
      {"0", "0"}
    };
    comms_types = comms_t;

    type_entry core_t[] = {
      {"Alias", "Alias"},
      {"Blank", "Blank"},
      {"BoolProperty", "BoolProperty"},
      {"Bool", "BoolProperty"},
      {"setBool", "BoolProperty.setBoolPropertyValue"},
      {"IntProperty", "IntProperty"},
      {"Int", "IntProperty"},
      {"setInt", "IntProperty.setIntPropertyValue"},
      {"DoubleProperty", "DoubleProperty"},
      {"Double", "DoubleProperty"},
      {"setDouble", "DoubleProperty.setDoublePropertyValue"},
      {"TextProperty", "TextProperty"},
      {"String", "TextProperty"},
      {"setString", "TextProperty.setTextPropertyValue"},
      {"RefProperty", "RefProperty"},
      {"Ref", "RefProperty"},
      {"setRef", "RefProperty.setRefPropertyValue"},
      {"Composite", "Composite"},
      {"NativeAction", "NativeAction"},
      {"NativeResource", "NativeResource"},
      {"List", "List"},
      {"Set", "Set"},
      {"Component", "Component"},
      {"Spike", "Component"},
      {"Binding", "Binding"},
      {"Watcher", "Watcher"},
      {"Connector", "Connector"},
      {"Assignment", "Assignment"},
      {"LibraryLoader", "LibraryLoader"},
      {"NativeThread", "NativeThread"},
      {"0", "0"}
    };
    core_types = core_t;

    type_entry display_t[] = {
      {"0", "0"}
    };
    display_types = display_t;

    type_entry files_t[] = {
      {"DirectoryIterator", "DirectoryIterator"},
      {"FileLocator", "FileLocator"},
      {"TextFileReader", "TextFileReader"},
      {"0", "0"}
    };
    files_types = files_t;

    type_entry gestures_t[] = {
      {"OneDollarClassifier", "OneDollarClassifier"},
      {"0", "0"}
    };
    gestures_types = gestures_t;

    type_entry gui_t[] = {
      {"Window", "Window"},
      {"Frame", "Frame"},
      {"Rectangle", "Rectangle"},
      {"Ellipse", "Ellipse"},
      {"Circle", "Circle"},
      {"Line", "Line"},
      {"Text", "Text"},
      {"RelativeText", "RelativeText"},
      {"Polyline", "Polyline"},
      {"Polygon", "Polygon"},
      {"PolyPoint", "Poly.addPointTo"},
      {"Path", "Path"},
      {"PathClosure", "PathClosure"},
      {"PathSubpath", "PathSubpath"},
      {"PathLine", "PathLine"},
      {"PathQuadratic", "PathQuadratic"},
      {"PathCubic", "PathCubic"},
      {"PathArc", "PathArc"},
      {"RectangleClip", "RectangleClip"},
      {"PathClip", "PathClip"},
      {"Image", "Image"},
      {"Group", "Group"},
      
      {"PixmapCache", "PixmapCache"},
      {"OutlineColor", "OutlineColor"},
      {"NoOutline", "NoOutline"},
      {"FillColor", "FillColor"},
      {"Texture", "Texture"},
      {"FillRule", "FillRule"},
      {"NoFill", "NoFill"},
      {"OutlineOpacity", "OutlineOpacity"},
      {"FillOpacity", "FillOpacity"},
      {"OutlineWidth", "OutlineWidth"},
      {"OutlineCapStyle", "OutlineCapStyle"},
      {"OutlineJoinStyle", "OutlineJoinStyle"},
      {"OutlineMiterLimit", "OutlineMiterLimit"},
      {"DashArray", "DashArray"},
      {"NoDashArray", "NoDashArray"},
      {"DashSubPattern", "DashSubPattern"},
      {"DashOffset", "DashOffset"},
      {"LinearGradient", "LinearGradient"},
      {"RadialGradient", "RadialGradient"},
      {"GradientStop", "GradientStop"},
      {"FontSize", "FontSize"},
      {"FontWeight", "FontWeight"},
      {"FontStyle", "FontStyle"},
      {"FontFamily", "FontFamily"},
      {"TextAnchor", "TextAnchor"},
      {"Translation", "Translation"},
      {"Rotation", "Rotation"},
      {"SkewX", "SkewX"},
      {"SkewY", "SkewY"},
      {"Scaling", "Scaling"},
      {"Homography", "Homography"},
      {"GradientTranslation", "GradientTranslation"},
      {"GradientRotation", "GradientRotation"},
      {"GradientSkewX", "GradientSkewX"},
      {"GradientSkewY", "GradientSkewY"},
      {"GradientScaling", "GradientScaling"},
      {"GradientHomography", "GradientHomography"},
      {"SimpleGradientTransform", "SimpleGradientTransform"},
      {"0", "0"}
    };
    gui_types = gui_t;

    type_entry input_t[] = {
      {"0", "0"}
    };
    input_types = input_t;

    type_entry macbook_t[] = {
      {"KeyboardLight", "KeyboardLight"},
      {"0", "0"}
    };
    macbook_types = macbook_t;

    type_entry modules_t[] = {
      {"LibraryLoader", "LibraryLoader"},
      {"0", "0"}
    };
    modules_types = modules_t;

    type_entry network_t[] = {
      {"0", "0"}
    };
    network_types = network_t;

    type_entry objects_t[] = {
      {"PhilipsHueBridge", "PhilipsHueBridge"},
      {"0", "0"}
    };
    objects_types = objects_t;

    type_entry phidgets_t[] = {
      {"0", "0"}
    };
    phidgets_types = phidgets_t;

    type_entry power_t[] = {
      {"0", "0"}
    };
    power_types = power_t;

    type_entry qtwidgets_t[] = {
      {"QPushButton", "QPushButton"},
      {"QCheckBox", "QCheckBox"},
      {"QRadioButton", "QRadioButton"},
      {"QLineEdit", "QLineEdit"},
      {"0", "0"}
    };
    qtwidgets_types = qtwidgets_t;

    type_entry sound_t[] = {
      {"Beep", "Beep"},
      {"PDPlugin", "PDPlugin"},
      {"0", "0"}
    };
    sound_types = sound_t;

    init ();
  }

  JTypeManager::~JTypeManager ()
  {
  }

}
