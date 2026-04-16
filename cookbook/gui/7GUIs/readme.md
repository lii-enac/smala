# 7GUIs

This is the smala version of the `7GUIs` [GUI Programming Benchmark](https://eugenkiss.github.io/7guis/) with a gradually increasing level of difficulty.

## 1- counter

Challenge: Understanding the basic ideas of a language/toolkit.

Counter serves as a gentle introduction to the basics of the language, paradigm and toolkit for one of the simplest GUI applications imaginable. Thus, Counter reveals the required scaffolding and how the very basic features work together to build a GUI application. A good solution will have almost no scaffolding.

go to [smala code](https://github.com/lii-enac/smala/tree/master/cookbook/gui/7GUIs/1-counter)

## 2- temp-converter

Challenges: bidirectional data flow, user-provided text input.

Temperature Converter increases the complexity of Counter by having bidirectional data flow between the Celsius and Fahrenheit inputs and the need to check the user input for validity. A good solution will make the bidirectional dependency very clear with minimal boilerplate code.

Temperature Converter is inspired by the [Celsius/Fahrenheit converter](https://www.artima.com/pins1ed/gui-programming.html#32.4) from the book Programming in Scala. It is such a widespread example—sometimes also in the form of a currency converter—that one could give a thousand references. The same is true for the Counter task.

go to [smala code](https://github.com/lii-enac/smala/tree/master/cookbook/gui/7GUIs/2-temp-converter)

## 3- flight-booker

Challenge: Constraints.

The focus of Flight Booker lies on modelling constraints between widgets on the one hand and modelling constraints within a widget on the other hand. Such constraints are very common in everyday interactions with GUI applications. A good solution for Flight Booker will make the constraints clear, succinct and explicit in the source code and not hidden behind a lot of scaffolding.

Flight Booker is directly inspired by the Flight Booking Java example in Sodium with the simplification of using textfields for date input instead of specialized date picking widgets as the focus of Flight Booker is not on specialized/custom widgets.

go to [smala code](https://github.com/lii-enac/smala/tree/master/cookbook/gui/7GUIs/3-flight-booker)

## 4- timer

Challenges: concurrency, competing user/signal interactions, responsiveness.

Timer deals with concurrency in the sense that a timer process that updates the elapsed time runs concurrently to the user’s interactions with the GUI application. This also means that the solution to competing user and signal interactions is tested. The fact that slider adjustments must be reflected immediately moreover tests the responsiveness of the solution. A good solution will make it clear that the signal is a timer tick and, as always, has not much scaffolding.

Timer is directly inspired by the timer example in the paper [Crossing State Lines: Adapting Object-Oriented Frameworks to Functional Reactive Languages](https://cs.brown.edu/people/sk/Publications/Papers/Published/ick-adapt-oo-fwk-frp/paper.pdf).

go to [smala code](https://github.com/lii-enac/smala/tree/master/cookbook/gui/7GUIs/4-timer)

## 5- CRUD

Challenges: separating the domain and presentation logic, managing mutation, building a non-trivial layout.

CRUD (Create, Read, Update and Delete) represents a typical graphical business application. The primary challenge is the separation of domain and presentation logic in the source code that is more or less forced on the implementer due to the ability to filter the view by a prefix. Traditionally, some form of MVC pattern is used to achieve the separation of domain and presentation logic. Also, the approach to managing the mutation of the list of names is tested. A good solution will have a good separation between the domain and presentation logic without much overhead (e.g. in the form of toolkit specific concepts or language/paradigm concepts), a mutation management that is fast but not error-prone and a natural representation of the layout (layout builders are allowed, of course, but would increase the overhead).

CRUD is directly inspired by the crud example in the blog post [FRP - Three principles for GUI elements with bidirectional data flow](https://apfelmus.nfshost.com/blog/2012/03/29-frp-three-principles-bidirectional-gui.html).

go to [smala code](https://github.com/lii-enac/smala/tree/master/cookbook/gui/7GUIs/5-CRUD)

## 6- circle-drawer

Challenges: undo/redo, custom drawing, dialog control*.

Circle Drawer’s goal is, among other things, to test how good the common challenge of implementing an undo/redo functionality for a GUI application can be solved. In an ideal solution the undo/redo functionality comes for free resp. just comes out as a natural consequence of the language / toolkit / paradigm. Moreover, Circle Drawer tests how dialog control*, i.e. keeping the relevant context between several successive GUI interaction steps, is achieved in the source code. Last but not least, the ease of custom drawing is tested.

\* Dialog control is explained in more detail in the paper [Developing GUI Applications: Architectural Patterns Revisited](https://ceur-ws.org/Vol-610/paper11.pdf) starting on page seven. The term describes the challenge of retaining context between successive GUI operations.

go to [smala code](https://github.com/lii-enac/smala/tree/master/cookbook/gui/7GUIs/6-circle-drawer)

## 7-cells

Challenges: change propagation, widget customization, implementing a more authentic/involved GUI application.

Cells is a more authentic and involved task that tests if a particular approach also scales to a somewhat bigger application. The two primary GUI-related challenges are intelligent propagation of changes and widget customization. Admittedly, there is a substantial part that is not necessarily very GUI-related but that is just the nature of a more authentic challenge. A good solution’s change propagation will not involve much effort and the customization of a widget should not prove too difficult. The domain-specific code is clearly separated from the GUI-specific code. The resulting spreadsheet widget is reusable.

Cells is directly inspired by the [SCells spreadsheet example](https://www.artima.com/pins1ed/the-scells-spreadsheet.html) from the book Programming in Scala. Please refer to the book (or the implementations in this repository) for more details especially with respect to the not directly GUI-related concerns like parsing and evaluating formulas and the precise syntax and semantics of the spreadsheet language.

go to [smala code](https://github.com/lii-enac/smala/tree/master/cookbook/gui/7GUIs/7-cells)


# TODO:
- assess with the [Cognitive Dimensions](https://eugenkiss.github.io/7guis/dimensions)
- assess by counting the number of lines that do not contain "// [7GUIs]" as a measure of boilerplate/scaffolding
  - without empty, comment-only and closing brace-only `}` lines 
```
find cookbook/gui/7GUIs/1-counter -name "*.sma" | xargs grep -v "\/\/ \[7GUIs\]" | grep -v "^\s*$" | grep -v "^\s*//" | grep -v –E "^\s*}+\s*" | wc -l
```

# FIXMEs and TODOs:

NOTE: on peut rajouter du code pour l'utilisabilité. ex: la cause du fait que le bouton du flight-booker ne passe pas dans l'état enable (date retour avant date aller)

## 1-counter:
- FIXME
  - TextField.text_color default values?
  - button label parameter should be after x and y(?)
    - maybe make it so that x,y,w,h are optional (inc. for SVG shapes): if they are to be controlled by a layout algorithm anyway, they will be modified

## 2-temp-converter
- FIXME
  - handle (partial) cycles
  - //regex_num_F = clone(regex_num_C)  // FIXME clone does not work
  - double click fait la sélection (comme le raccourci du Ctrl+A)

## 3-flight-booker
- FIXME
  - tests & feedback dès l'édition (key press)
  - "01/01/2025" =: T1.text does not work 
  - ZOrderedGroup zog {            // FIXME this is for the combobox, since its popup inner listbox has to be on top of other widgets
  - // FIXME why isn't the combobox inited with value 1?
  - // FIXME the text fields shrink when selecting an entry in the combo box !!!!???
  - // FIXME? could be cb_model.items[1] or cb_model.one_way -> T2 and cb_model.return_flight ->! T2
  - // FIXME T2 non-editing mode resembles the disabled state. TextEdit style is weird anyway.
  - Tab ne valide pas le champ de texte (feedback d'erreur)


## 4-timer
- FIXME
  - The bv stuff is intrusive, we know too much of the internal of the widget
  - mouseTracking = 1    // FIXME otherwise Slider adjustment won't work
  - permettre de choisir le nombre de chiffres après la virgules pour les double

## 5-CRUD
- FIXME
  - pas de fond sur les list box item
  - calage des textes des LBI à gauche
  - Wrapper pour is_empty

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