# Comparison of several GUI architectures

- MVC - Model View Controller
- PAC - Presentation Abstraction Controller
- MVVM - Model View ViewModel 
- MDPC - Model DisplayView PickingView (Inverse)Transform Controller
- MVP - Model View Presenter (TODO)
- MVzmC - Model ViewZone ViewModel Controller (TODO) 


To better see the differences, on macOS:

`opendiff cookbook/archi/mvc cookbook/archi/pac`

`opendiff cookbook/archi/mvc cookbook/archi/mvvm`

`opendiff cookbook/archi/mvc cookbook/archi/mdpc`



## The problem with MVC:

```The first Swing prototype followed a traditional MVC separation in which each component had a separate model object and delegated its look-and-feel implementation to separate view and controller objects. We quickly discovered that this split didn't work well in practical terms because the view and controller parts of a component required a tight coupling (for example, it was very difficult to write a generic controller that didn't know specifics about the view). So we collapsed these two entities into a single UI (user-interface) object, as shown in this diagram.``` 
https://www.oracle.com/java/technologies/a-swing-architecture.html

```Figure 4: Essential dependencies between model, view, and controller. (I call this essential because in fact the view and controller do link to each other directly, but developers mostly don't use this fact.)```
https://www.martinfowler.com/eaaDev/uiArchs.html



## References


### MVC

Reenskaug, Trygve M. H. MVC. XEROX PARC 1978-79.
https://folk.universitetetioslo.no/trygver/themes/mvc/mvc-index.html

Steve Burbeck. Applications Programming in Smalltalk-80: How to use Model-View-Controller (MVC). 1987, 1992.
https://www.researchgate.net/publication/238719652_Applications_programming_in_smalltalk-80_how_to_use_model-view-controller_mvc

Glenn E. Krasner and Stephen T. Pope. A Description of the Model-View-Controller User Interface Paradigm in the Smalltalk-80 System. 1988.
http://iihm.imag.fr/blanch/ens/2006-2007/RICM3/IHM/documents/krasner-MVC.pdf

Glenn E. Krasner and Stephen T. Pope. 1988. A cookbook for using the model-view controller user interface paradigm in Smalltalk-80. J. Object Oriented Program. 1, 3 (Aug./Sept. 1988), 26–49.
https://www.lri.fr/~mbl/ENS/FONDIHM/2013/papers/Krasner-JOOP88.pdf


### PAC
Coutaz, Joëlle (1987): PAC, an Object-Oriented Model for Dialog Design. In: Bullinger, Hans-Jorg, Shackel, Brian (eds.) INTERACT 87 - 2nd IFIP International Conference on Human-Computer Interaction September 1-4, 1987, Stuttgart, Germany. pp. 431-436. 

https://www.lri.fr/~mbl/ENS/FONDIHM/2013/papers/Coutaz-Interact87.pdf

### MVP
MVP: Model-View-Presenter. The Taligent Programming Model for C++ and Java. 1996. http://www.wildcrest.com/Potel/Portfolio/mvp.pdf

https://en.wikipedia.org/wiki/Model%E2%80%93view%E2%80%93presenter

### MVVM

John Gossman. Introduction to Model/View/ViewModel pattern for building WPF apps. 2005.
https://learn.microsoft.com/en-us/archive/blogs/johngossman/introduction-to-modelviewviewmodel-pattern-for-building-wpf-apps

Josh Smith. Patterns - WPF Apps With The Model-View-ViewModel Design Pattern. 2009. https://learn.microsoft.com/en-us/archive/msdn-magazine/2009/february/patterns-wpf-apps-with-the-model-view-viewmodel-design-pattern

https://en.wikipedia.org/wiki/Model%E2%80%93view%E2%80%93viewmodel

### MDPC

Conversy, S., Barboni, E., Navarre, D., Palanque, P. "Improving modularity of interactive software with the MDPC architecture". In Proceedings of EIS (Engineering Interactive Systems) conference 2007, joint HCSE 2007, EHCI 2007 and DSVIS 2007 conferences, Lecture Notes in Computer Science, pp321-338. Springer Verlag, 2007.
http://recherche.enac.fr/~conversy/research/papers/eis2007-MDPC.pdf


Conversy, S. Improving Usability of Interactive Graphics Specification and Implementation with Picking Views and Inverse Transformations. In VL/HCC 2011: Proceedings of the Symposium on Visual Languages and Human-Centric Computing (VL/HCC), pp153-160. IEEE, 2011.
http://recherche.enac.fr/~conversy/research/papers/vlhcc2011-MDPC.pdf

http://recherche.enac.fr/~conversy/research/MDPC/index.html

### comparisons
What's a controller anyway?
http://wiki.c2.com/?WhatsaControllerAnyway..



Martin Fowler. GUI Architectures. 2006. https://www.martinfowler.com/eaaDev/uiArchs.html

https://geekswithblogs.net/gwbarchive/mvvm-compared-to-mvc-and-mvp/

https://manojjaggavarapu.wordpress.com/2012/05/02/presentation-patterns-mvc-mvp-pm-mvvm/


Notes: MVVM is precluded by 'Presentation Model' by Martin Fowler (abstracting a model of the view), which itself seems a reinvention of the 'logic view' of the Seeheim model.
