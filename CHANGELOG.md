# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),

smala is strongly linked to djnn-cpp developpement : <https://github.com/lii-enac/djnn-cpp>

## [1.21.0] - 2024-07-15

### compliant with djnn-cpp [1.21.0]

    - please see CHANGELOG.md of djnn-cpp for more in-depth details changes

### NEW

    - Layer Rotation: Added rotation support to the Layer recipe in the cookbook.
    - Layer Customization: Introduced layer_custom to explain how to customize Layer for specific cases.
    - MVVM Pattern: Introduced a new version of the MVVM pattern with detailed examples.
    - Buildroot: Added a README for using buildroot in standalone projects.

### Added

    - Cookbook Enhancements:
        - Implemented new recipe for MVVM.
        - Added a recipe "ivyDie" to send Ivy's send_die message and modified helloIvy recipe to respond to Ivy's die message.
        - MVC recipe: Improved the MVC code example by adding separate controllers and renaming variables for clarity.
        
    - Standalone Project:
        - Improved scripts for buildroot and added better user alerts.
        - Added a CtrlAllSupp script to kill the app executing KA command.

    - Editor Modes: Added the keyword "setRef" to the editor modes.

### Changed

    - Build Scripts: Improved Linux compilation process and fixed typos in various scripts.
    - Makefile Updates:
        - Enhanced support for cross-compilation and older compilers.
        - Improved handling of multiple directories in ld_library_path.
        - Renamed CXXFLAGS to DJNN_CXXFLAGS for cross-compilation use.
        - Made smalac independent of config.mk.

### Removed

    - Removed unused classes and attic code from the MVC example.

### Fixed

    - Compilation Issues:
        - Fixed various compilation issues on Linux/Windows platforms.
        - Corrected z-order issues in the library.
        - Addressed typo errors in pkgdeps.mk.
        - Fixed the use of remove child "from" parent and added it to the Smala VSCode syntax.

    - MVVM and MVC Patterns:
        - Resolved issues related to view and model updates.
        - Fixed interaction bugs in the MVC example, including border interactions and selection interaction.

    - Cookbook Fixes:
        - Corrected compilation errors in the SVG and keyboard color examples.
        - Fixed picking view in the MDPC example.
        - Updated transition syntax in the PAC and MVC examples.

## [1.20.0] - 2023-10-10

### compliant with djnn-cpp [1.20.0]

    - please see CHANGELOG.md of djnn-cpp for more in-depth details changes

### NEW

    - NEW debug option: _DEBUG_SEE_RECOMPUTE_PIXMAP_AND_PAINTEVENT 
    - NEW debug option: _DEBUG_SEE_RECOMPUTE_PIXMAP_ONLY
    - NEW debug option: _DEBUG_SEE_COMPONENTS_DESTRUCTION_INFO_LEVEL = 0|1|2 to track warning on component destruction
    - NEW debug option: _DEBUG_NO_TOUCH_EVENT to enable touch events (especially for macos trackpad) default seting: 1
        - SO we have to activate multitouch to use it
    - NEW debug option: _DEBUG_SEE_ACTIVATION_SEQUENCE_TARGET_LOCATION = file.sma:lineno
    - NEW Cookbook recipe:
        - Chat

### Added

    - Added support to fast compilation

### Changed

    - Improved cookbook recipes :
        - scrollbar
        - multi_layer
        - window recipe to have transparent frame
        - video : local camera by default, stream on OBS
        - tabManager code
    - Improved config.mk : move sanitizer options into config.default.mk (and so config.mk)
    - Improve support for mold
    - Avoid need for iostream with print()
    - Improved generation of DELETE_CONTENT
    - Improved code of stand_alone

### Deprecated

    - schedule_delete -> schedule_deletion
    - in code generation: Parent/FatChildProcess -> CoreProcess

### Fixed

    - Fixed abstract_property.h inclusion
    - Fixed libsmala install_name
    - Fixed debug info and line number for native
    - Fixed standalone push button
    - Fixed *.mk for the use of V=max
    - Fixed linux compilation
    - Fixed editor mode path on windows
    - Fixed cookbook recipes:
        - boxes_and_arrows
        - swim
        - paused_control

## [1.19.0] - 2023-01-26

### compliant with djnn-cpp [1.19.0]

    - please see CHANGELOG.md of djnn-cpp for more in-depth details changes

### NEW

    - use of the new z_order.
    - NEW lib widget and cookbook: ListViewer
    - NEW support double unit
    - NEW cookbooks: template_property, double_array, load_then_clone, boxes_and_arrows, multi_layers
    - NEW Added a smala compilation flag -gen-cleaner to generate memory cleaner at the end of generated code
    - NEW Added $SMAFLAG in config.default.mk to use option: -g, -gen-cleaner
    - NEW support for Mold
    - NEW added deb_git target for daily deb packaging

### Changed

    - Fixed cook
    - Improved Stand_alone
    - Improved cookbook: boxes, simple_zoom, tile_grid, argv, Dial, tab, layout
    - Improved lib widget: Checkbox, HSlider, VSlider, UITextField, PushButton, standAloneComboBox, DropDownMenu
    - Improved Cpp Builder
    - Improved Makefile

### Deprecated

    - replaced deprecated loadFromXML by either load_from_XML or load_from_XML_once

## [1.18.0] - 2022-06-09

### compliant with djnn-cpp [1.18.0]

    - please see CHANGELOG.md of djnn-cpp for more in-depth details changes

### NEW

    - Improve Smala syntax editor mode for vscode : please update using editor_mode/vscode/README.md
    - New cookbook recipes : handpick, console
    - NEW cookbook recipes : bidi (experimental), ROS2 communication (disable by default), video (stream, rtsp - disable by default)
    - NEW experimental mode - _AUTHORIZE_CYCLE = 1|0
    - NEW auto-interactive script test using : pyautogui -- Create NEW git branch : interactive_test_suite
    - NEW Stand_alone scheme : make stand_alone 
    - NEW LazyConnector Component : =?>
    - NEW LazyAssignment Component :  =?:
    - NEW Component: GenericKeyboard
    - NEW widgets: UITextField, StandAloneSlider, StandAlonePushButton, StandAloneToggleButton, StandAlonecomboBox, HSpace, VSpace

### Changed

    - Improved delete_content code generator by using scheduled_delete
    - Combobox can have zero items, and it is possible to add items one per one
    - Force to use the pixel unit for font. Allows to have the same rendering on MAC & Linux
    - AssignmentSequence can be named when defined in a binding
    - Improves lib widgets: Animator, Combobox, HBox, VBox, VSlider, HSlider, PushButton, Label, ToggleButton, TextLineEdit
    - Improves cookbook recipes : Layer
    - slider component : allow the external modification of the value
    - Improved debug messages.
    - Improved Readme.md
    - Cleaned Code, comments

### Removed

    - REMOVED WidgetContainer widget -- not needed anymore !

## [1.17.0] - 2022-01-05

### compliant with djnn-cpp [1.17.0]

    - please see CHANGELOG.md of djnn-cpp for more in-depth details changes

### NEW

    - Added support for M1 Macos
    - NEW smala :
        - Added kind of inheritance mechanism between user componant : eq:  Buttons inherits the Widget
        - Added new syntax for addChildrentTo: eq. addChildrentTo parent { item1, item2, item3}
        - Added support to pyautogui for auto interaction test : python script and exemples
        - Added a mechanism to pass a function as an argument and the type NativeCode in component definition
        - Added support argc argv for _main_ process
    - NEW smala component and cookbook : 
        - Deref bool, text
        - Previous : double, int text
        - HBox and VBox componants and there cookbooks
        - WidgetContainer component
        - layout/boxes cookbook
        - Collision :  add a collision detection cookbook
        - Argv cookbook
    - NEW in lib smala: 
        - svg mockup for widget design
        - Checkbox
        - Toggle button
        - Checkbox manager for exclusive choice
        - simple horizontal slider widget
        - Label Widget
        - ComboBox widget

### Added

    - Enabled anonymous insert
    - Added self assignment operators

### Changed

    - Improved cookbook:
        - Layer: padding
        - Dash Array with double
        - Swims
    - Improved lib/widget component:
        - Button --> PushButton
        - RadioButton
    - Improved compilation dependencies
    - Improved Install.md
    - Cleaned code

### Fixed

    - Fixed cookbooks:
        - Buttons
        - Clock

## [1.16.0] - 2021-07-28

### compliant with djnn-cpp [1.&6.0]

    - please see CHANGELOG.md of djnn-cpp for more in-depth details changes

note: debut on changelog. This section has to be completed, if we have time.

### NEW

    - NEW Component : please refer to djnn CHANGELOG
        - SimpleTextEdit, SimpleText
        - DirectoryObserver
        - LazyAssignment : do not trigger if the new value is the same old value : symbol to use '=?:'
    - NEW smala component : 
        - GraphEdgeAdder
     - NEW keywords :
        - 'delete_content' to use on containers
        - '%' modulo operator
        - Added labels for fsm transitions: 'toto: idle -> state1'
    - NEW lib/widget component :
        - button
    - NEW cookbooks : 
        - layer
        - fill_and_delete_content
        - share_cursor
        - timed_automaton
    - NEW editor_mode :
        - Added vscode plugin: syntax enlightening
        - Improved sublime support
    - NEW support for vscode.
    - NEW debug options that can be called directly from smala code :
        - see DEBUG.md for more details
        - _DEBUG_SEE_COLOR_PICKING_VIEW
        - _DEBUG_GRAPH_CYCLE_DETECT
        - _DEBUG_SEE_ACTIVATION_SEQUENCE
        - _DEBUG_SEE_ACTIVATION_SEQUENCE_TARGET_TIME_US
        - _DEBUG_SEE_ACTIVATION_SEQUENCE_ONLY_TARGETED  

### Changed

    - Improved packages dependency 
    _ Improved packages build system
    - Improved lib/widget component :
        - Animator

### Fixed

    - Fixed documentation
    - Fixed pkgconfig file generation
    - Fixed cookbooks :
        - physics
        - window
        - Client
        - async_native
        - nativeactions
        - native_collection_action
        - Clone
        - sketching_simple/advanced
        - fitts_law
        - checkbox
        - strip_board
        - DropDownMenu
        - tab
        - regex
        - Clock

## [1.15.0] - 2021-01-28

### compliant with djnn-cpp [1.15.0]

    - please see CHANGELOG.md of djnn-cpp for more in-depth details changes

### NEW

    - NEW start working on javascript code generation from smala : javascript builder
        - Improved generic builder.
        - Improved code consistency in smala so in cpp builder.
        - Fixed all cookbook recipes, app to match the changes.
    - Lots of works to reduce compilation time : clean headers, precompiled headers, dependencies.
        BECAREFUL: developer has to be more precise on includes declaration when using C++ code. (eg: C++ native)
        You can use the auto-generated file build/src/process_class_path.cpp to find which header as to be call for each class/typedef ...
    - NEW cookbook recipe: The clock made by J.-L. Vinot.
    - NEW cookbook recipe: imperative 
    - NEW cookbook recipe: incremental json parsing with djnn concepts
    - NEW cookbook recipe: javascript.
    - NEW Makefile for Stand_Alone with precompiled_header option : use_pch = yes

### Added

### Changed

    - Fixed all cookbook recipes, app to match the builder changes.
    - Changed smalac: clean, and improve code.
    - Changed smalac: surround simple instruction in switch with component
    - Improved cookbook recipe : swim with offline demo.
    - Improved cookbook recipe for physics.
    - Simplest cookbook recipe is now simpler.

### Fixed

    - Fixed dnd cookbook recipe.
    - Fixed Linux/MacOs/Windows compilation and dependencies in consequences of the work on the headers

## [1.14.0] - 2020-11-23

### compliant with djnn-cpp [1.14.0]

    - please see CHANGELOG.md of djnn-cpp for more in-depth details changes

### NEW

    - Package for windows (and Linux) are avaluable on : https://github.com/lii-enac/smala/releases
    - Added SVG recipe, workflow and preferences to use in smala
    - NEW Client/server (TCP/IP) cookbook recipes.
    - NEW audio/animation cookbook recipe.
    - NEW auddio/visual cookbook recipe.
    - NEW Physics cookbook recipes.
    - NEW screenshots using WallClock cookbook recipes.
    - NEW window cookbook recipe.
    - NEW SWIM connection cookbook recipe (aeronautical)

### Added

### Changed

    - Better debug info - line number fixed - add more parser info to context.
    - Improved Fitts Law recipe.
    - Improved warning on Switch generation
    - Improved and clean-up Makefile
    - Improved audio/midi cookbook recipe.
    - Improved async_native cookbook recipe.
    - Improved emscripten support

### Fixed

    - Fixed context generation for ELSEIF
    - Fixed notification animation

## [1.13.0] - 2020-10-07

### compliant with djnn-cpp [1.13.0]

    - please see CHANGELOG.md of djnn-cpp for more in-depth details changes

### NEW

    - NEW native collection component + cookbook 
    - New DropDownMenu recipe in gui widget libsmala
    - NEW "for : "  loop operator (ex: "for r : rectangles")
    - NEW opérator : " ~>  " allowingg user to add edges in the casual graph 
    - NEW support for multi Connector
    - NEW support for multi assignment
    - NEW - libsmala - animation manager component +  easing cookbook recipe
    - NEW support for screenshot service throug Frame/window + screenshot cookbook recipe
    - NEW blinking cookbook recipe
    - NEW simultaneous_contrast cookbook recipe
    - NEW simultaneous_color_contrast cookbook recipe
    - NEW - libsmala - Control key manager 
    - NEW Keyboard cookbook 
    - NEW curl native async cookbook recipe
    - NEW compilation with C++17
    - NEW widget menu fold cookbook recipe

### Added

    - Added cookbook crazyflie recipe

### Changed

    - Added - libsmala - Button interface
    - Simplified async_native recipe
    - Move PanAndZoom into libsmala
    - Updated cookbook recipes from djnn-cpp 1.13.0 api
    - Changed zoom recipe : mouseCentered by default
    - Improved and cleaned up code
    - Improved Makefile : cross_prefix, install-pkgdeps
    - Changed Static_cast for dynamic_cast in 

### Removed

    - Removed all buildpath reference
    - Removed : mainloop is no more an existing global variable
    - Removed fsm_guard 
    - Commented midi component recipe (not working on Windows)

### Fixed

    - Fixed few bugs in TextLineEdit
    - Fixed the use of wheel->dx, dy
    - Fixed some Linux compilation
    - Fixed Sorter recipe
    - Fixed the use of other_runtime_lib_path in Makefile

## [1.12.0] - skipped

## [1.11.0] - 2020-06-25

### compliant with djnn-cpp [1.12.0]

    - please see CHANGELOG.md of djnn-cpp for more in-depth details changes

### NEW

    - NEW supopr for the new process from djnn-cpp : totally transparent for end-user
    - NEW Generate .deb package for ubuntu/debian
    - NEW potentionmeter widget component into smalalib
    - NEW cookbook using Async NativeAction

### Added

    - Added cookbook using potentionmeter

### Changed

    - IvyAcess component is now special with special constructor
    - Improved build_find
    - Cleaned code and cookbook
    - Improved button design into smalalib

### Fixed

    - Fixed Pan and zoom cookbook
    - Fixed cookbook midi
    - Fixed a bug in cookbook dock

## [1.10.0] - 2020-05-07

### compliant with djnn-cpp [1.11.0]

    - please see CHANGELOG.md of djnn-cpp for more in-depth details changes

### NEW

    - NEW MAJOR REFACTORING - see the HowTo below
        - Path as p.1 must be written p.[1]
        - It is now possible to write: p.[<expr>] as 
            ex.: p.[i+1] = 10
        - toString is a function that takes a process as argument and returns a literal string
            ex.: string s = toString (p)
        - String concatenation can be done with the + operator but the first term must be explicitly a string
            ex.: "foo" + p + "bar" => t
            ex.: toString(p) + "bar" => t
        - the comparison between processes requires an explicit cast
            ex.: &p1 == &p2
        - Addchild has been remove and replace by : <<
            ex.: a << b
            ex.: a << myfunc () // such as clone, or loadFromXML ...
        - the use of "$" (get_value() of a int/bool/double property ) has changed. 
            $(expression) is now directly written $expression.
            ex.: $(foo.bar) ==> $foo.bar
          The sign is not needed when it is obvious : 
            ex.: sqrt(foo.angle)
          But it is needed when you call constructors :
            ex: My_rectangle rect ($foo.x, $foo.y, 100, 100)
        - Added anonymous assignmentSequence : a -> { b =: c }
        - (for now) the use a the "dump" function has changed. cannot specify the dump level 
            ex.: dump foo
        - You can ask for the source of a native action using keyword : _src_
            ex.: rect.press -> (root) {
                foo = _src_
            }
        - Improved EOL management : should use "\" sign if a instruction is write on several lines.
        - Added support of "0x" prefix for int (hexa)
        - (test) added new keyword : _keep_ to keep the name of a c++ variable

    - NEW libsmala.dylib/so : a lib with widgets, behavior and others shared code.
        - just start need to be fill. for now : simpleDrag, button
    - NEW added install rule and pkg-config support for libsmala in Makefile
    - NEW Drag'n Drop (dnd) cookbook recipe 
    - NEW dynamic_rectangle cookbook recipe
    - NEW midi cookbook recipe
    - NEW brew tab for macOs

### Changed

    - the use of "$" (get_value() of a int/bool/double property ) has changed. 
            $(expression) is now directly written $expression.
            ex.: $(foo.bar) ==> $foo.bar
          The sign is not needed when it is obvious : 
            ex.: sqrt(foo.angle)
          But it is needed when you call constructors :
            ex: My_rectangle rect ($foo.x, $foo.y, 100, 100)
    - Renamed cookbook/network ==> cookbook/comms
    - Improved nativeaction cookbook recipe
    - Improved editor_mode for sublime_text: toString, _src_
    - made finalize_construction public now in class NativeExpressionAction, no need to generate it
    - Cleaned code

### Deprecated

    - (for now) the use a the "dump" function has changed. cannot specify the dump level 
            ex.: dump foo

### Removed

    - AddChild has been remove and replace by : <<

### Fixed

    - Fixed grammar ambiguous rules

## [1.9.0] - 2018-12-17

### compliant with djnn-cpp [1.10.0]

    - please see CHANGELOG.md of djnn-cpp for more in-depth details changes

### NEW

    - NEW cookbook recipe on crazyflie
    - New cookbook recipe on Regex

### Added

    - Makefile: depend on djnn-cpp pkg-conf
    - Makefile: added "make install" and "make install prefix=" rules
    - Makefile: new tree architecture in build_dir

### Changed

    - Changed find_component -> fond_child
    - Changed cookbook: RefProperty
    - Changed config.default.mk : Please delete your config.mk
    - Cleaned Makefike of any Java legacy and C legacy

### Deprecated

### Removed

    - Removed useless pinch_zoom cookbook

### Fixed

    - Fixed standalone Makefile
    - Fixed the fake_root
    - Fixed headers generation
    - Fixed error macro
    - Fixed compilation
    - Fixed some clean rules

## [1.8.0] - 2020-03-13

### compliant with djnn-cpp [1.9.0]

    - please see CHANGELOG.md of djnn-cpp for more in-depth details changes

### NEW

    - NEW allow break, continue and return statements
    - NEW the symbol '|->'' for activator :  |-> spike
    - NEW cookbook: layout
    - NEW cookbook: audio
    - New cookbook: mainloop
    - NEW root and syshook auto-magically run ! : no need to run root and run syshook at the end

### Added

    - Added 'mspf' child to windows component: millisecond per frame informations
    - Enabled path in move child
    - Adapte to new djnn module: exec_env AND auto-intialize

### Changed

    - Improved editor_mode for SublimeText
    - Improved DoubleClick recipe
    - Improved code
    - Made cookbooks quit on window closing

### Fixed

    - Fixed parenting mechanism in main program as in dynamic program (AddChildrenTo)
    - Fixed HelloIvy cookbook to work on every platform
    - Fixed naming branch in SwitchRange

## [1.7.0] - 2020-01-31

### compliant with djnn-cpp [1.8.0]

    - please see CHANGELOG.md of djnn-cpp for more in-depth details changes

### NEW

    - NEW cookbook for a simple Menu widget
    - NEW cookbook illustrating the use of a Sorter
    - NEW support for XMLSerialize
    - NEW mode for IDE: emacs
    - NEW allow hexadecimal notation for int (eq: #23efFF)
    - NEW support for html color names
    - NEW support for djnn audio module
    - NEW animation cookbook
    - NEW line edition cookbook
    - NEW cookbook for Texture

### Added

    - Enable expressions in bindings
    - Start support for emscripten
    - Manage display module import
    - Improved Stand_alone template.
    - Added a guard and error message on addChildrenTo (builder)

### Deprecated

    - Deprecated isString to toString

### Fixed

    - Fixed drag_pan_zoom cookbook.
    - Fixed seg_fault in code generation
    - Fixed new compatibility with djnn 1.8.0
    - Fixed an issue with find_child generation
    - Fixed Makefile
    - Fixed typo and clean-up code

## [1.6.1] - 2019-11-19 [YANKED]

### compliant with djnn-cpp [1.7.0]

    - Please see CHANGELOG.md of djnn-cpp for more in-depth details changes

### Fixed

    - Fixed a bug in path building
    - Fixed typo

## [1.6.0] - 2019-11-18

### compliant with djnn-cpp [1.7.0]

    - please see CHANGELOG.md of djnn-cpp for more in-depth details changes

### NEW

    - NEW cookbook recipe: text field edit
    - NEW use of djnn constant by using prefix DJN
    - NEW addition of a new way to access children by their index (eg. list)

### Added

    - Added Makefile sanatizer options 

### Changed

    - Replaced delete by schelude_delete (djnn) in cpp_builder code generator
    - Improved README for windows instruction
    - Improved cookbook recipes : sketching, checkbox, fitts_laws

## [1.5.0] - 2019-09-17

### compliant with djnn-cpp [1.6.0]

    - please see CHANGELOG.md of djnn-cpp for more in-depth changes

### NEW

    - New operator: "isString ()"" to help compilator generating string from property 
        eg : isString ( "incr:" + incr.state) => tp.text
    - New operator: "&" to help compiler generate pointer from property
        eg: t = getRef (&root.f.touches.$added), setref (&root.touch.key, &t)
    - New operator:  "$"  to help compilator get value from a property (except: string and pointer) 
        can directly use the double value of a Double in something else
        eg: Double d(10), Rectangle ($d, $d, 100, 100, 0, 0) // = Rectangle (10, 10, 100, 100, 0, 0)
    - New can now directly call any C function if declared in _native_code_ bracket. No need CCall anymore
    - New native action mechanism (perf)
    - New Switch_range component suport
    - New cookbook recipes: scrollbar, multi_touch, multi_touch_drag, multi_touch_rrr, multi_touch_rrr_dyn
    - New cookbook recipes: switch_range
    - New Phigdets component support

### Added

### Changed

    - All arguments in _define_ component definition has to be Process instead of Component
    - refactoring grammar
    - Improved cookbook recipes : bindings, display, sketching_advanced, drag_pan_zoom
    - Improved editor_mode
    - Improved Makefile 
    - cleaned code

### Fixed

    - Fixed windows compilation

### Broken

    - cookbook : pinch_zoom, rotate_resize

## [1.4.0] - 2019-06-04

### compliant with djnn-cpp [1.5.0]

    - please see CHANGELOG.md of djnn-cpp for more in-depth details changes

### NEW

    - Added and manage automatically synchronizer 
    - Added new operator for connector with a first copy on activation "=:>""
    - Setting a refproperty will automatically use pointer : ref r (foo), if foo is component will automatically get foo pointer address.
    - Setting double into Doublepropperty is automatic : Double d(0.0) ; d = 5.0
    - Setting int into Intpropperty is automatic : Int i(0.0) ; i = 5
    - Allowed to define new variables in native
    - Added "if else" pattern in action
    - Added "for" loop in native actions
    - Added while loop in native actions
    - Added increment and decrement : using symbol --  and ++ : eg: root.foo++
    - Added accumulated_transforms cookbook recipe
    - Added rotate_resize recipe cookbook recipe
    - Added pinch to zoom cookbook recipe
    - Added fsm with guards cookbook recipe
    - Added multi_touch_drag cookbook recipe
    - Added a simple print function : print()
    - Added few tests

### Added

    - Managed switch_list component properly
    - By default initialize display when using gui
    - Improved warning compilation

### Changed

    - "=>" is now a connector component without a first copy on activation, react only on triggering
    - Removed named assignment mechanism ... use AssigmentSequence instead : eg: 
        .notify_selected = index =: tabManager.selected_index : 1 ==>  AssignmentSequence notify_selected (1) { index =: tabManager.selected_index }
    - Improved editor_mode for SublimeText :
        .improves syntax highlighting for smala - added better Type, ::, ::>, =:> and change keyword
        .detect imperative code
    - Improved drag & pan cookbook recipe to accumulation in a single Homography
    - Update all cookbook with new mechanism : =>, =:> ...

### Removed

    - Removed SetDouble, 
    - Removed named assignment mechanism ... use AssigmentSequence instead

### Fixed

    - Fixed several cookbook recipes using new mechanism
    - Fixed smala parser
    - Fixed compilation issues on Linux platform
    - Fixed a shift/reduce conflict

## [1.3.0] - 2019-04-05

### compliant with djnn-cpp [1.4.0]

    - please see CHANGELOG.md of djnn-cpp for more in-depth details changes

### New

    - NEW display API NEEDED to use Frame component
        eg: use display 
            Frame f ("mywindow")
    - NEW componant - GenericMouse
    - NEW cookbook recipe - refProperty (dynamicly)
    - NEW cookbook recipe - multi-touch
    - NEW editor_mode for sublime_text : comments, functions
    - New mode : hidePointer
    - Can now store componant directly into Ref
        eg: myComponent => myRef

    ### Changed
    - Instructions now can have parameters : delete, dump, notify, run, stop
    - Improved cookbook recipe - simple_touch
    - Improved native expressions efficiency
    - Improved Makefile

## [1.2.0] - 2019-03-15

### compliant with djnn-cpp [1.3.0]

    - please see CHANGELOG.md of djnn-cpp for more in-depth details changes 

### NEW

    - NEW cookbook - zoom recipe (frame-centered and mouse-centered)

### Added

    - added NEW smala expressions for "anti-binding": !->, ->!, !->!
        eg: A !-> B when A is desactivated, B is Activated
        eg: A ->! B when A is Activated, B is desactivated
        eg: A !->! B when A is desactivated, B is desactivated

### Changed

    - improved Makefile: clean up and mechanism, allow sub-directories in import
    - improved code generation for: delete
    - improved README.md and INSTALL.md
    - improved and update cookbook: stand_alone, pan_and_zoom, pan_and_zoom_and_drag
    - improved : smala.sublime-syntax
    - improve Windows (10/8) support

### Removed

    - removed warning on unknown module

## [1.1.1] - 2019-02-04 [YANKED]

### Added

    - CHANGELOG.md

## [1.1.0] - 2019-02-01

### compliant with djnn-cpp [1.1.0] and [1.2.0]

    - please see CHANGELOG.md of djnn-cpp for more in-depth details changes

### Added

    - API - new kind of binding: ->, ->!, !->, !->!
    - API - support for dash-array
    - COOKBOOK - new recipe on binding

## [1.0.0] - 2018-12-17

### compliant with djnn-cpp [1.0.0]

    - please see CHANGELOG.md of djnn-cpp for more in-depth details changes

note: debut on changelog. This section has to be completed, if we have time.

### NEW

### Added

### Changed

### Deprecated

### Removed

### Fixed
