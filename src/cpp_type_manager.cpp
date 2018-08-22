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

#include "cpp_type_manager.h"

#include <iostream>

namespace Smala {


  CPPTypeManager::CPPTypeManager () : TypeManager ()
  {
    type_entry coreGS[] = {
      {"syshook", "system_hook"},
      {"0", "0"}
    };
    coreGlobalSymbols = coreGS;

    type_entry displayGS[] = {
//      {"Displays", "djnDisplays"},
      {"0", "0"}
    };
    displayGlobalSymbols = displayGS;

    type_entry inputGS[] = {
      {"InputDevices", "djnInputDevices"},
      {"Mice", "djnMice"},
      {"TouchPanels", "djnTouchPanels"},
#if 0
      {"Styluses", "djnStyluses"},
      {"RawPointers", "djnRawPointers"},
      {"Joysticks", "djnJoysticks"},
      {"RotaryKnobs", "djnRotaryKnobs"},
      {"Absolute3DDevices", "djnAbsolute3DDevices"},
      {"MakeyMakeys", "djnMakeyMakeys"},
      {"Keyboards", "djnKeyboards"},
#endif
      {"0", "0"}
    };
    inputGlobalSymbols = inputGS;

    type_entry phidgetsGS[] = {
//      {"InterfaceKits", "djnInterfaceKits"},
      {"0", "0"}
    };
    phidgetsGlobalSymbols = phidgetsGS;

    type_entry powerGS[] = {
 //     {"Batteries", "djnBatteries"},
      {"0", "0"}
    };
    powerGlobalSymbols = powerGS;

    type_entry guiGS[] = {
      {"mouseTracking", "mouse_tracking"},
      {"fullScreen", "full_screen"},
      {"0", "0"}
    };
    guiGlobalSymbols = guiGS;

    type_entry soundGS[] = {
      //{"PortAudio", "djnPortAudio"},
      {"0", "0"}
    };
    soundGlobalSymbols = soundGS;

    type_entry animation_t[] = {
      {"SlowInSlowOutInterpolator","SlowInSlowOutInterpolator"},
      {"0", "0"}
    };
    animation_types = animation_t;


    type_entry base_t[] = {
      {"Connector", "Connector"},
      {"PausedConnector", "PausedConnector"},
      {"Clock", "Clock"},
      {"FSM", "FSM"},
      {"FSMState", "FSMState"},
      {"FSMTransition", "FSMTransition"},
      {"Switch", "Switch"},
      {"Or", "Or"},
      {"Xor", "Xor"},
      {"And", "And"},
      {"Not", "Not"},
      {"Adder", "Adder"},
      {"AdderAccumulator", "AdderAccumulator"},
      {"AscendingComparator", "AscendingComparator"},
      {"Divider", "Divider"},
      {"EqualityComparator", "EqualityComparator"},
      {"Incr", "Incr"},
      {"Modulo", "Modulo"},
      {"Multiplier", "Multiplier"},
      {"SignInverter", "SignInverter"},
      {"Previous", "Previous"},
      {"StrictAscendingComparator", "StrictAscendingComparator"},
      {"Subtractor", "Subtractor"},
      {"HermiteCurve", "HermiteCurve"},
      {"Exp", "Exp"},
      {"Log", "Log"},
      {"Log10", "Log10"},
      {"Pow", "Pow"},
      {"Sqrt", "Sqrt"},
      {"Abs", "Abs"},
      {"Min", "Min"},
      {"Max", "Max"},
      {"BoundedValue", "BoundedValue"},
//      {"Normalizer", "djnCreateNormalizer"},
      {"ArcCosine", "ArcCosine"},
      {"ArcSine", "ArcSine"},
      {"ArcTangent", "ArcTangent"},
      {"ArcTangent2", "ArcTangent2"},
      {"Cosine", "Cosine"},
      {"HyperbolicArcCosine", "HyperbolicArcCosine"},
      {"HyperbolicArcSine", "HyperbolicArcSine"},
      {"HyperbolicArcTan", "HyperbolicArcTan"},
      {"HyperbolicCosine", "HyperbolicCosine"},
      {"HyperbolicSine", "HyperbolicSine"},
      {"HyperbolicTangent", "HyperbolicTangent"},
      {"Sine", "Sine"},
      {"Tangent", "Tangent"},
//      {"RandomGenerator", "djnCreateRandomGenerator"},
//      {"Formula", "djnCreateFormula"},
//      {"IntToDouble", "djnCreateIntToDouble"},
      {"TextCatenator", "TextCatenator"},
      {"DoubleFormatter", "DoubleFormatter"},
      {"TextPrinter", "TextPrinter"},
      {"TextComparator", "TextComparator"},
//      {"LogPrinter", "djnCreateLogPrinter"},
//      {"RegexpFilter", "djnCreateRegexpFilter"},
//      {"PN", "djnCreatePN"},
//      {"PNPlace", "djnCreatePNPlace"},
//      {"PNTransition", "djnCreatePNTransition"},
//      {"PNInputTransition", "djnCreatePNInputTransition"},
//      {"PNArc", "djnCreatePNArc"},
//      {"Noop", "djnCreateNoop"},
//      {"Counter", "djnCreateCounter"},
 //     {"JsonSerializeComponent", "djnJsonSerializeComponent"},
      {"0", "0"}
    };
    base_types = base_t;

    type_entry comms_t[] = {
      {"IvyAccess", "IvyAccess"},
      {"0", "0"}
    };
    comms_types = comms_t;

    type_entry core_t[] = {
      {"Alias", "alias"},
      {"Activator", "Activator"},
      {"Blank", "Spike"},
      {"Spike", "Spike"},
      {"BoolProperty", "BoolProperty"},
      {"Bool", "BoolProperty"},
      {"setBool", "set_value"},
      {"IntProperty", "IntProperty"},
      {"Int", "IntProperty"},
      {"setInt", "set_value"},
      {"DoubleProperty", "DoubleProperty"},
      {"Double", "DoubleProperty"},
      {"setDouble", "set_value"},
      {"TextProperty", "TextProperty"},
      {"String", "TextProperty"},
      {"setString", "set_value"},
      {"RefProperty", "RefProperty"},
      {"Ref", "RefProperty"},
      {"setRef", "set_value"},
      {"Exit", "Exit"},
//      {"Composite", "djnCreateComposite"},
      {"NativeAction", "NativeAction"},
 //     {"NativeResource", "djnCreateNativeResource"},
      {"List", "List"},
      {"Set", "Set"},
      {"Component", "Component"},
      {"AssignmentSequence", "AssignmentSequence"},
      {"Binding", "Binding"},
//      {"Watcher", "djnCreateWatcher"},
      {"Assignment", "Assignment"},
      {"PausedAssignment", "PausedAssignment"},
//      {"LibraryLoader", "djnCreateLibraryLoader"},
//      {"NativeThread", "djnCreateNativeThread"},
      {"0", "0"}
    };
    core_types = core_t;

    type_entry display_t[] = {
      {"0", "0"}
    };
    display_types = display_t;

    type_entry files_t[] = {
//      {"DirectoryIterator", "djnCreateDirectoryIterator"},
//      {"FileLocator", "djnCreateFileLocator"},
//      {"TextFileReader", "djnCreateTextFileReader"},
      {"0", "0"}
    };
    files_types = files_t;

    type_entry gestures_t[] = {
//      {"OneDollarClassifier", "djnCreateOneDollarClassifier"},
      {"0", "0"}
    };
    gestures_types = gestures_t;

    type_entry gui_t[] = {
      {"Window", "Window"},
      {"Frame", "Window"},
      {"Rectangle", "Rectangle"},
      {"Ellipse", "Ellipse"},
      {"Circle", "Circle"},
      {"Line", "Line"},
      {"Text", "Text"},
      {"RelativeText", "RelativeText"},
      {"Polyline", "Polyline"},
      {"Polygon", "Polygon"},
      {"PolyPoint", "PolyPoint"},
      {"Path", "Path"},
      {"PathClosure", "PathClosure"},
      {"PathSubpath", "PathMove"},
      {"PathMove", "PathMove"},
      {"PathLine", "PathLine"},
      {"PathQuadratic", "PathQuadratic"},
      {"PathCubic", "PathCubic"},
      {"PathArc", "PathArc"},
      {"RectangleClip", "RectangleClip"},
      {"PathClip", "PathClip"},
      {"Image", "Image"},
      {"Group", "Group"},
//      {"PixmapCache", "djnCreateGUIPixmapCache"},
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
 //     {"KeyboardLight", "djnCreateKeyboardLight"},
      {"0", "0"}
    };
    macbook_types = macbook_t;

    type_entry modules_t[] = {
//      {"LibraryLoader", "djnCreateLibraryLoader"},
      {"0", "0"}
    };
    modules_types = modules_t;

    type_entry network_t[] = {
      {"0", "0"}
    };
    network_types = network_t;

    type_entry objects_t[] = {
//      {"PhilipsHueBridge", "djnCreatePhilipsHueBridge"},
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
 //     {"QPushButton", "djnCreateQPushButton"},
 //     {"QCheckBox", "djnCreateQCheckBox"},
 //     {"QRadioButton", "djnCreateQRadioButton"},
 //     {"QLineEdit", "djnCreateQLineEdit"},
      {"0", "0"}
    };
    qtwidgets_types = qtwidgets_t;

    type_entry sound_t[] = {
//      {"Beep", "djnCreateBeep"},
      {"0", "0"}
    };
    sound_types = sound_t;

    init ();
  }

  CPPTypeManager::~CPPTypeManager ()
  {
  }

}
