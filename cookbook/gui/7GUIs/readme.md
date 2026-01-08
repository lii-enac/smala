This is the smala version of the [7GUIs GUI Programming Benchmark](https://eugenkiss.github.io/7guis/)

# TODO:
- assess with the [Cognitive Dimensions](https://eugenkiss.github.io/7guis/dimensions)
- assess by counting the number of lines that do not contain "// [7GUIs]" as a measure of boilerplate/scaffolding
  - without empty, comment-only and closing brace-only `}` lines 
```
find cookbook/gui/7GUIs/1-counter -name "*.sma" | xargs grep -v "\/\/ \[7GUIs\]" | grep -v "^\s*$" | grep -v "^\s*//" | grep -v –E "^\s*}+\s*" | wc -l
```

# FIXMEs and TODOs:



## 1-counter:
- FIXME
  - TextField.text_color default values?
  - button label parameter should be after x and y(?)
    - maybe make it so that x,y,w,h are optional (inc. for SVG shapes): if they are to be controlled by a layout algorithm anyway, they will be modified

## 2-temp-converter
- FIXME
  - handle (partial) cycles
  - //regex_num_F = clone(regex_num_C)  // FIXME clone does not work

## 3-flight-booker
- FIXME
  - "01/01/2025" =: T1.text does not work 
  - ZOrderedGroup zog {            // FIXME this is for the combobox, since its popup inner listbox has to be on top of other widgets
  - // FIXME why isn't the combobox inited with value 1?
  - // FIXME the text fields shrink when selecting an entry in the combo box !!!!???
  - // FIXME? could be cb_model.items[1] or cb_model.one_way -> T2 and cb_model.return_flight ->! T2
  - // FIXME T2 non-editing mode resembles the disabled state. TextEdit style is weird anyway.


## 4-timer
- FIXME
  - Gauge widget (here we use a scale)
  - The bv stuff is intrusive, we know too much of the internal of the widget
  - mouseTracking = 1    // FIXME otherwise Slider adjustment won't work


## 5-CRUD
*Not* working
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


## 7-cells
Operations:
  - click in a cell (say B4), the edit box should move into the cell, enter "=sum(A2:A7"), enter, B4 should display 0
  - click in cell A3, enter 32, B4 should be updated to 32
  - click in cell A5, enter 5, B4 should now display 37 

- FIXME
  - z order, why does the edit box is under the cell boundaries??
    - we thus have to resize the cell bg...
  - coordinates of text, edit box etc. makes it difficult to display things on top of other ones
  - 
  - the status of the interaction goes into limbo sometimes
  - add 'mean' as a function (and others...)
  - Bindings could be created in the smala program, not in the C++ formula parsing code
  - List instead of Component, otherwise the symtable contains the last created cell (they all have the same name)


## other stuff:
  - Rename Component into Composite
  - rename 'Native'/lambda (it's called like that in the parser :-/) into something more appropriate
  - maybe capture more variables into 'native'
  - implement mbl's sticky line algorithms