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
      {"fullScreen", "full_screen"},
      {"mouseTracking", "mouse_tracking"},
      {"0", "0"}
    };
    guiGlobalSymbols = guiGS;

    type_entry soundGS[] = {
      //{"PortAudio", "djnPortAudio"},
      {"0", "0"}
    };
    soundGlobalSymbols = soundGS;

    type_entry animation_t[] = {
      {"Oscillator", "Oscillator"},
      {"SlowInSlowOutInterpolator","SlowInSlowOutInterpolator"},
      {"0", "0"}
    };
    animation_types = animation_t;


    type_entry base_t[] = {
//      {"RegexpFilter", "djnCreateRegexpFilter"},
//      {"PN", "djnCreatePN"},
//      {"PNPlace", "djnCreatePNPlace"},
//      {"PNTransition", "djnCreatePNTransition"},
//      {"PNInputTransition", "djnCreatePNInputTransition"},
//      {"PNArc", "djnCreatePNArc"},
//      {"Noop", "djnCreateNoop"},
 //     {"JsonSerializeComponent", "djnJsonSerializeComponent"},
      {"Abs", "Abs"},
      {"Adder", "Adder"},
      {"AdderAccumulator", "AdderAccumulator"},
      {"And", "And"},
//      {"Normalizer", "djnCreateNormalizer"},
      {"ArcCosine", "ArcCosine"},
      {"ArcSine", "ArcSine"},
      {"ArcTangent", "ArcTangent"},
      {"ArcTangent2", "ArcTangent2"},
      {"AscendingComparator", "AscendingComparator"},
      {"BoundedValue", "BoundedValue"},
      {"Clock", "Clock"},
      {"Connector", "Connector"},
      {"Cosine", "Cosine"},
      {"Counter", "Counter"},
      {"Divider", "Divider"},
      {"DoubleFormatter", "DoubleFormatter"},
      {"EqualityComparator", "EqualityComparator"},
      {"Exp", "Exp"},
      {"FSM", "FSM"},
      {"FSMState", "FSMState"},
      {"FSMTransition", "FSMTransition"},
      {"HermiteCurve", "HermiteCurve"},
      {"HyperbolicArcCosine", "HyperbolicArcCosine"},
      {"HyperbolicArcSine", "HyperbolicArcSine"},
      {"HyperbolicArcTan", "HyperbolicArcTan"},
      {"HyperbolicCosine", "HyperbolicCosine"},
      {"HyperbolicSine", "HyperbolicSine"},
      {"HyperbolicTangent", "HyperbolicTangent"},
      {"Incr", "Incr"},
      {"Log", "Log"},
      {"Log10", "Log10"},
      {"LogPrinter", "LogPrinter"},
      {"Max", "Max"},
      {"Min", "Min"},
      {"Modulo", "Modulo"},
      {"Multiplier", "Multiplier"},
      {"Not", "Not"},
      {"Or", "Or"},
      {"PausedConnector", "PausedConnector"},
      {"Pow", "Pow"},
      {"Previous", "Previous"},
      {"SignInverter", "SignInverter"},
      {"Sine", "Sine"},
      {"Sqrt", "Sqrt"},
      {"StrictAscendingComparator", "StrictAscendingComparator"},
      {"Subtractor", "Subtractor"},
      {"Switch", "Switch"},
      {"SwitchList", "SwitchList"},
      {"Tangent", "Tangent"},
//      {"RandomGenerator", "djnCreateRandomGenerator"},
//      {"Formula", "djnCreateFormula"},
//      {"IntToDouble", "djnCreateIntToDouble"},
      {"TextCatenator", "TextCatenator"},
      {"TextComparator", "TextComparator"},
      {"TextPrinter", "TextPrinter"},
      {"Xor", "Xor"},
      {"0", "0"}
    };
    base_types = base_t;

    type_entry comms_t[] = {
      {"IvyAccess", "IvyAccess"},
      {"0", "0"}
    };
    comms_types = comms_t;

    type_entry core_t[] = {
      {"Activator", "Activator"},
      {"Alias", "alias"},
//      {"Watcher", "djnCreateWatcher"},
      {"Assignment", "Assignment"},
      {"AssignmentSequence", "AssignmentSequence"},
      {"Binding", "Binding"},
      {"Blank", "Spike"},
      {"Bool", "BoolProperty"},
      {"BoolProperty", "BoolProperty"},
      {"Component", "Component"},
      {"Double", "DoubleProperty"},
      {"DoubleProperty", "DoubleProperty"},
      {"Exit", "Exit"},
      {"Int", "IntProperty"},
      {"IntProperty", "IntProperty"},
 //     {"NativeResource", "djnCreateNativeResource"},
      {"List", "List"},
      {"ListIterator", "ListIterator"},
//      {"Composite", "djnCreateComposite"},
      {"NativeAction", "NativeAction"},
      {"PausedAssignment", "PausedAssignment"},
      {"Ref", "RefProperty"},
      {"RefProperty", "RefProperty"},
      {"Set", "Set"},
      {"setBool", "set_value"},
      {"setDouble", "set_value"},
      {"setInt", "set_value"},
      {"SetIterator", "SetIterator"},
      {"setRef", "set_value"},
      {"setString", "set_value"},
      {"Spike", "Spike"},
      {"String", "TextProperty"},
      {"TextProperty", "TextProperty"},
      {"Timer", "Timer"},
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
      {"Circle", "Circle"},
      {"DashArray", "DashArray"},
      {"DashOffset", "DashOffset"},
      {"DashSubPattern", "DashSubPattern"},
      {"Ellipse", "Ellipse"},
      {"FillColor", "FillColor"},
      {"FillOpacity", "FillOpacity"},
      {"FillRule", "FillRule"},
      {"FontFamily", "FontFamily"},
      {"FontSize", "FontSize"},
      {"FontStyle", "FontStyle"},
      {"FontWeight", "FontWeight"},
      {"Frame", "Window"},
      {"GradientHomography", "GradientHomography"},
      {"GradientRotation", "GradientRotation"},
      {"GradientScaling", "GradientScaling"},
      {"GradientSkewX", "GradientSkewX"},
      {"GradientSkewY", "GradientSkewY"},
      {"GradientStop", "GradientStop"},
      {"GradientTranslation", "GradientTranslation"},
      {"Group", "Group"},
      {"Homography", "Homography"},
      {"Image", "Image"},
      {"LCHToRGBConverter", "LCHToRGBConverter"},
      {"Line", "Line"},
      {"LinearGradient", "LinearGradient"},
      {"NoDashArray", "NoDashArray"},
      {"NoFill", "NoFill"},
      {"NoOutline", "NoOutline"},
      {"OutlineCapStyle", "OutlineCapStyle"},
//      {"PixmapCache", "djnCreateGUIPixmapCache"},
      {"OutlineColor", "OutlineColor"},
      {"OutlineJoinStyle", "OutlineJoinStyle"},
      {"OutlineMiterLimit", "OutlineMiterLimit"},
      {"OutlineOpacity", "OutlineOpacity"},
      {"OutlineWidth", "OutlineWidth"},
      {"Path", "Path"},
      {"PathArc", "PathArc"},
      {"PathClip", "PathClip"},
      {"PathClosure", "PathClosure"},
      {"PathCubic", "PathCubic"},
      {"PathLine", "PathLine"},
      {"PathMove", "PathMove"},
      {"PathQuadratic", "PathQuadratic"},
      {"PathSubpath", "PathMove"},
      {"Polygon", "Polygon"},
      {"Polyline", "Polyline"},
      {"PolyPoint", "PolyPoint"},
      {"RadialGradient", "RadialGradient"},
      {"Rectangle", "Rectangle"},
      {"RectangleClip", "RectangleClip"},
      {"RelativeText", "RelativeText"},
      {"RGBToLCHConverter", "RGBToLCHConverter"},
      {"Rotation", "Rotation"},
      {"Scaling", "Scaling"},
      {"SimpleGradientTransform", "SimpleGradientTransform"},
      {"SkewX", "SkewX"},
      {"SkewY", "SkewY"},
      {"Text", "Text"},
      {"TextAnchor", "TextAnchor"},
      {"Texture", "Texture"},
      {"Translation", "Translation"},
      {"Window", "Window"},
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
