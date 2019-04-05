# Changelog
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),


## IMPORTANT

smala is deeply link to djnn-cpp developpement : https://github.com/lii-enac/djnn-cpp


## [Unreleased]
### Changed
- None



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


