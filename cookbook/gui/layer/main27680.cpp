#include "exec_env/exec_env-dev.h"
#include "core/utils/error.h" // for Context class
#undef error // avoid name clash with error macro and possible following #include
#undef warning // avoid name clash with error macro and possible following #include

using namespace djnn;


#line 1 "cookbook/gui/layer/main.cpp"

#line 16
#include "core/core-dev.h"

#line 17
#include "base/base-dev.h"

#line 18
#include "display/display-dev.h"

#line 19
#include "gui/gui-dev.h"

#line 21
int
main () {
	init_core ();
	init_exec_env ();
	init_base ();
	init_display ();
	init_gui ();

#line 22
Context::instance()->parser_info(22, 1, 22, 15, "/Users/conversy/recherche/istar/code/djnn/smala/cookbook/gui/layer/main.sma");
	auto* cpnt_0 = new Component (nullptr, "root");

#line 23
Context::instance()->parser_info(23, 3, 23, 10, "/Users/conversy/recherche/istar/code/djnn/smala/cookbook/gui/layer/main.sma");
	auto* cpnt_1 = new Window (cpnt_0, "f", "layer", 0, 0, 600, 600);

#line 24
Context::instance()->parser_info(24, 3, 24, 10, "/Users/conversy/recherche/istar/code/djnn/smala/cookbook/gui/layer/main.sma");
	auto* cpnt_2 = new Exit (cpnt_0, "ex", 0, 1);

#line 25
Context::instance()->parser_info(25, 3, 25, 16, "/Users/conversy/recherche/istar/code/djnn/smala/cookbook/gui/layer/main.sma");
	new Binding (cpnt_0, "", cpnt_1->find_child ("close"), ACTIVATION, cpnt_2, ACTIVATION);

#line 27
Context::instance()->parser_info(27, 3, 27, 18, "/Users/conversy/recherche/istar/code/djnn/smala/cookbook/gui/layer/main.sma");
	auto* cpnt_3 = new OutlineWidth (cpnt_0, "ow", 10);

#line 28
Context::instance()->parser_info(28, 3, 28, 15, "/Users/conversy/recherche/istar/code/djnn/smala/cookbook/gui/layer/main.sma");
	auto* cpnt_4 = new FillColor (cpnt_0, "fc", 255, 0, 0);

#line 29
Context::instance()->parser_info(29, 3, 29, 17, "/Users/conversy/recherche/istar/code/djnn/smala/cookbook/gui/layer/main.sma");
	auto* cpnt_5 = new OutlineColor (cpnt_0, "_", 0, 0, 255);

#line 32
Context::instance()->parser_info(32, 3, 32, 11, "/Users/conversy/recherche/istar/code/djnn/smala/cookbook/gui/layer/main.sma");
	auto* cpnt_6 = new Layer (cpnt_0, "bg");

#line 33
Context::instance()->parser_info(33, 5, 33, 11, "/Users/conversy/recherche/istar/code/djnn/smala/cookbook/gui/layer/main.sma");
	auto* cpnt_7 = new Text (cpnt_6, "_", 0, 20, "this text and the star are in a layer, while the circle is moving on top");

#line 34
Context::instance()->parser_info(34, 5, 34, 11, "/Users/conversy/recherche/istar/code/djnn/smala/cookbook/gui/layer/main.sma");
	auto* cpnt_8 = new Text (cpnt_6, "_", 0, 30, "a clock applies every 2s a small x translation on the star and thus invalidates the layer");

#line 35
Context::instance()->parser_info(35, 5, 35, 18, "/Users/conversy/recherche/istar/code/djnn/smala/cookbook/gui/layer/main.sma");
	auto* cpnt_9 = new Translation (cpnt_6, "t", - 500, 0);

#line 36
Context::instance()->parser_info(36, 5, 36, 8, "/Users/conversy/recherche/istar/code/djnn/smala/cookbook/gui/layer/main.sma");
