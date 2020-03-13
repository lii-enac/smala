# Changelog
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),


## IMPORTANT

smala is strongly linked to djnn-cpp developpement : https://github.com/lii-enac/djnn-cpp

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
    - Fixed an issue with find_component generation
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

