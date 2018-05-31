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

#include "c_type_manager.h"

#include <iostream>

namespace Smala {


  CTypeManager::CTypeManager () : TypeManager ()
  {
    type_entry coreGS[] = {
      {"ComponentSetAdded", "djnComponentSetAdded"},
      {"ComponentSetRemoved", "djnComponentSetRemoved"},
      {"syshook", "djnSystemHook"},
      {"mainloop", "djnMainLoop"},
      {"0", "0"}
    };
    coreGlobalSymbols = coreGS;

    type_entry displayGS[] = {
      {"Displays", "djnDisplays"},
      {"0", "0"}
    };
    displayGlobalSymbols = displayGS;

    type_entry inputGS[] = {
      {"InputDevices", "djnInputDevices"},
      {"Mice", "djnMice"},
      {"TouchPanels", "djnTouchPanels"},
      {"Styluses", "djnStyluses"},
      {"RawPointers", "djnRawPointers"},
      {"Joysticks", "djnJoysticks"},
      {"RotaryKnobs", "djnRotaryKnobs"},
      {"Absolute3DDevices", "djnAbsolute3DDevices"},
      {"MakeyMakeys", "djnMakeyMakeys"},
      {"Keyboards", "djnKeyboards"},
      {"0", "0"}
    };
    inputGlobalSymbols = inputGS;

    type_entry phidgetsGS[] = {
      {"InterfaceKits", "djnInterfaceKits"},
      {"0", "0"}
    };
    phidgetsGlobalSymbols = phidgetsGS;

    type_entry powerGS[] = {
      {"Batteries", "djnBatteries"},
      {"0", "0"}
    };
    powerGlobalSymbols = powerGS;

    type_entry guiGS[] = {
      {"mouseTracking", "djn_QtMouseTracking"},
      {"0", "0"}
    };
    guiGlobalSymbols = guiGS;

    type_entry soundGS[] = {
      {"PortAudio", "djnPortAudio"},
      {"0", "0"}
    };
    soundGlobalSymbols = soundGS;

    type_entry animation_t[] = {
      {"SlowInSlowOutInterpolator","djnCreateSlowInSlowOutInterpolator"},
      {"0", "0"}
    };
    animation_types = animation_t;

    type_entry base_t[] = {
      {"Clock", "djnCreateClock"},
      {"Exit", "djnCreateExit"},
      {"FSM", "djnCreateFSM"},
      {"FSMState", "djnCreateFSMState"},
      {"FSMTransition", "djnCreateFSMTransition"},
      {"Switch", "djnCreateSwitch"},
      {"Or", "djnCreateOr"},
      {"Xor", "djnCreateXor"},
      {"And", "djnCreateAnd"},
      {"Not", "djnCreateNot"},
      {"Incr", "djnCreateIncr"},
      {"Pow", "djnCreatePow"},
      {"Adder", "djnCreateAdder"},
      {"Subtractor", "djnCreateSubtractor"},
      {"Multiplier", "djnCreateMultiplier"},
      {"Divider", "djnCreateDivider"},
      {"SignInverter", "djnCreateSignInverter"},
      {"EqualityComparator", "djnCreateEqualityComparator"},
      {"AscendingComparator", "djnCreateAscendingComparator"},
      {"StrictAscendingComparator", "djnCreateStrictAscendingComparator"},
      {"AdderAccumulator", "djnCreateAdderAccumulator"},
      {"Normalizer", "djnCreateNormalizer"},
      {"HermiteCurve", "djnCreateHermiteCurve"},
      {"Sine", "djnCreateSine"},
      {"Cosine", "djnCreateCosine"},
      {"Tangent", "djnCreateTangent"},
      {"RandomGenerator", "djnCreateRandomGenerator"},
      {"Formula", "djnCreateFormula"},
      {"IntToDouble", "djnCreateIntToDouble"},
      {"TextCatenator", "djnCreateTextCatenator"},
      {"IntFormatter", "djnCreateIntFormatter"},
      {"DoubleFormatter", "djnCreateDoubleFormatter"},
      {"TextPrinter", "djnCreateTextPrinter"},
      {"TextComparator", "djnCreateTextComparator"},
      {"LogPrinter", "djnCreateLogPrinter"},
      {"RegexpFilter", "djnCreateRegexpFilter"},
      {"PN", "djnCreatePN"},
      {"PNPlace", "djnCreatePNPlace"},
      {"PNTransition", "djnCreatePNTransition"},
      {"PNInputTransition", "djnCreatePNInputTransition"},
      {"PNArc", "djnCreatePNArc"},
      {"Noop", "djnCreateNoop"},
      {"Counter", "djnCreateCounter"},
      {"JsonSerializeComponent", "djnJsonSerializeComponent"},
      {"0", "0"}
    };
    base_types = base_t;

    type_entry comms_t[] = {
      {"IvyAccess", "djnCreateIvyAccess"},
      {"IvyBus", "djnCreateIvyBus"},
      {"IvyExporter", "djnCreateIvyExporter"},
      {"IvyImporter", "djnCreateIvyImporter"},
      {"0", "0"}
    };
    comms_types = comms_t;

    type_entry core_t[] = {
      {"Alias", "djnCreateAlias"},
      {"Blank", "djnCreateBlank"},
      {"BoolProperty", "djnCreateBoolProperty"},
      {"Bool", "djnCreateBoolProperty"},
      {"setBool", "djnSetBoolProperty"},
      {"IntProperty", "djnCreateIntProperty"},
      {"Int", "djnCreateIntProperty"},
      {"setInt", "djnSetIntProperty"},
      {"DoubleProperty", "djnCreateDoubleProperty"},
      {"Double", "djnCreateDoubleProperty"},
      {"setDouble", "djnSetDoubleProperty"},
      {"TextProperty", "djnCreateTextProperty"},
      {"String", "djnCreateTextProperty"},
      {"setString", "djnSetTextProperty"},
      {"RefProperty", "djnCreateRefProperty"},
      {"Ref", "djnCreateRefProperty"},
      {"setRef", "djnSetRefProperty"},
      {"Composite", "djnCreateComposite"},
      {"NativeAction", "djnCreateNativeAction"},
      {"NativeResource", "djnCreateNativeResource"},
      {"List", "djnCreateList"},
      {"Set", "djnCreateSet"},
      {"Component", "djnCreateComponent"},
      {"Spike", "djnCreateComponent"},
      {"Binding", "djnCreateBinding"},
      {"Watcher", "djnCreateWatcher"},
      {"Connector", "djnCreateConnector"},
      {"PausedConnector", "djnCreatePausedConnector"},
      {"Assignment", "djnCreateAssignment"},
      {"PausedAssignment", "djnCreatePausedAssignment"},
      {"LibraryLoader", "djnCreateLibraryLoader"},
      {"NativeThread", "djnCreateNativeThread"},
      {"0", "0"}
    };
    core_types = core_t;

    type_entry display_t[] = {
      {"0", "0"}
    };
    display_types = display_t;

    type_entry files_t[] = {
      {"DirectoryIterator", "djnCreateDirectoryIterator"},
      {"FileLocator", "djnCreateFileLocator"},
      {"TextFileReader", "djnCreateTextFileReader"},
      {"0", "0"}
    };
    files_types = files_t;

    type_entry gestures_t[] = {
      {"OneDollarClassifier", "djnCreateOneDollarClassifier"},
      {"0", "0"}
    };
    gestures_types = gestures_t;

    type_entry gui_t[] = {
      {"Window", "djnCreateWindow"},
      {"Frame", "djnCreateGUIFrame"},
      {"Rectangle", "djnCreateGUIRectangle"},
      {"Ellipse", "djnCreateGUIEllipse"},
      {"Circle", "djnCreateGUICircle"},
      {"Line", "djnCreateGUILine"},
      {"Text", "djnCreateGUIText"},
      {"RelativeText", "djnCreateGUIRelativeText"},
      {"Polyline", "djnCreateGUIPolyline"},
      {"Polygon", "djnCreateGUIPolygon"},
      {"PolyPoint", "djnCreateGUIPolyPoint"},
      {"Path", "djnCreateGUIPath"},
      {"PathClosure", "djnCreateGUIPathClosure"},
      {"PathSubpath", "djnCreateGUIPathSubpath"},
      {"PathLine", "djnCreateGUIPathLine"},
      {"PathQuadratic", "djnCreateGUIPathQuadratic"},
      {"PathCubic", "djnCreateGUIPathCubic"},
      {"PathArc", "djnCreateGUIPathArc"},
      {"RectangleClip", "djnCreateGUIRectangleClip"},
      {"PathClip", "djnCreateGUIPathClip"},
      {"Image", "djnCreateGUIImage"},
      {"Group", "djnCreateGUIGroup"},
      {"PixmapCache", "djnCreateGUIPixmapCache"},
      {"OutlineColor", "djnCreateOutlineColor"},
      {"NoOutline", "djnCreateNoOutline"},
      {"FillColor", "djnCreateFillColor"},
      {"Texture", "djnCreateTexture"},
      {"FillRule", "djnCreateFillRule"},
      {"NoFill", "djnCreateNoFill"},
      {"OutlineOpacity", "djnCreateOutlineOpacity"},
      {"FillOpacity", "djnCreateFillOpacity"},
      {"OutlineWidth", "djnCreateOutlineWidth"},
      {"OutlineCapStyle", "djnCreateOutlineCapStyle"},
      {"OutlineJoinStyle", "djnCreateOutlineJoinStyle"},
      {"OutlineMiterLimit", "djnCreateOutlineMiterLimit"},
      {"DashArray", "djnCreateDashArray"},
      {"NoDashArray", "djnCreateNoDashArray"},
      {"DashSubPattern", "djnCreateDashSubPattern"},
      {"DashOffset", "djnCreateDashOffset"},
      {"LinearGradient", "djnCreateLinearGradient"},
      {"RadialGradient", "djnCreateRadialGradient"},
      {"GradientStop", "djnCreateGradientStop"},
      {"FontSize", "djnCreateFontSize"},
      {"FontWeight", "djnCreateFontWeight"},
      {"FontStyle", "djnCreateFontStyle"},
      {"FontFamily", "djnCreateFontFamily"},
      {"TextAnchor", "djnCreateTextAnchor"},
      {"Translation", "djnCreateTranslation"},
      {"Rotation", "djnCreateRotation"},
      {"SkewX", "djnCreateSkewX"},
      {"SkewY", "djnCreateSkewY"},
      {"Scaling", "djnCreateScaling"},
      {"Homography", "djnCreateHomography"},
      {"GradientTranslation", "djnCreateGradientTranslation"},
      {"GradientRotation", "djnCreateGradientRotation"},
      {"GradientSkewX", "djnCreateGradientSkewX"},
      {"GradientSkewY", "djnCreateGradientSkewY"},
      {"GradientScaling", "djnCreateGradientScaling"},
      {"GradientHomography", "djnCreateGradientHomography"},
      {"SimpleGradientTransform", "djnCreateSimpleGradientTransform"},
      {"0", "0"}
    };
    gui_types = gui_t;

    type_entry input_t[] = {
      {"0", "0"}
    };
    input_types = input_t;

    type_entry macbook_t[] = {
      {"KeyboardLight", "djnCreateKeyboardLight"},
      {"0", "0"}
    };
    macbook_types = macbook_t;

    type_entry modules_t[] = {
      {"LibraryLoader", "djnCreateLibraryLoader"},
      {"0", "0"}
    };
    modules_types = modules_t;

    type_entry network_t[] = {
      {"0", "0"}
    };
    network_types = network_t;

    type_entry objects_t[] = {
      {"PhilipsHueBridge", "djnCreatePhilipsHueBridge"},
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
      {"QPushButton", "djnCreateQPushButton"},
      {"QCheckBox", "djnCreateQCheckBox"},
      {"QRadioButton", "djnCreateQRadioButton"},
      {"QLineEdit", "djnCreateQLineEdit"},
      {"0", "0"}
    };
    qtwidgets_types = qtwidgets_t;

    type_entry sound_t[] = {
      {"Beep", "djnCreateBeep"},
      {"PDPlugin", "djnCreatePDPlugin"},
      {"0", "0"}
    };
    sound_types = sound_t;

    init ();
  }

  CTypeManager::~CTypeManager ()
  {
  }

}
