This is the smala version of the [7GUIs GUI Programming Benchmark](https://eugenkiss.github.io/7guis/)

# TODO:
- assess with the [Cognitive Dimensions](https://eugenkiss.github.io/7guis/dimensions)
- assess by counting the number of lines that do not contain "// [7GUIs]" as a measure of boilerplate/scaffolding
  - without empty, comment-only and closing brace-only `}` lines 
```
find cookbook/gui/7GUIs/1-counter -name "*.sma" | xargs grep -v "\/\/ \[7GUIs\]" | grep -v "^\s*$" | grep -v "^\s*//" | grep -v "^\s*\}\s*" | wc -l
```

# FIXMEs and TODOs:



## 1-counter:
- FIXME
  - frame should have default values
  - TextField.text_color default values

## 2-temp-converter
- FIXME
  - layout API
  - handle (partial) cycles

## 3-flight-booker
- TODO
  - validate dates
  - init dates
  
- FIXME
  - combobox not inited
  - validation only if T2 is active


## 4-timer
- TODO

- FIXME
  - Gauge widget (here we use a scale)
  - The bv stuff is intrusive, we know too much of the internal of the widget
  - mouseTracking = 1    // FIXME otherwise Slider adjustment won't work


## 5-CRUD
*Not working*
- TODO
    - ListBox
    - everything


## 6-circle-drawer
- FIXME
  - mouseTracking = 1 // for enter and leave events FIXME should be automatic
  - DropDownMenu seems to be a ComboBox?!
  - cannot create inline component with another, deeper native, - the compiler error seems pointless
  - popup window
  - selection management
  - undoing creation necessitates multiple triggering
- IDEAS
  - could we propose a typical ChangePropertyAction that takes a property, an old value and a new value?
