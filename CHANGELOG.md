# Changelog
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),


## IMPORTANT

smala is deeply link to djnn-cpp developpement : https://github.com/lii-enac/djnn-cpp


## [Unreleased]
### Changed
- None


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
-  Managed switch_list component properly
-  By default initialize display when using gui
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
note: debut on changelog. This section has to be completed, if we have time. 
### NEW
### Added
### Changed
### Deprecated
### Removed
### Fixed
### Security


