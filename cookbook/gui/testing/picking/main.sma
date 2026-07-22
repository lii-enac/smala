/*
*  Smala cookbook picking
*
*  The copyright holders for the contents of this file are:
*    Ecole Nationale de l'Aviation Civile, France (2026)
*  See file "license.terms" for the rights and conditions
*  defined by copyright holders.
*
*
*  Contributors:
*     Vincent Peyruqueou <vincent.peyruqueou@enac.fr>
*
*/
use core
use base
use display
use gui

//import gui.DebugD
import core.property.text_property // getString
import exec_env.main_loop


_native_code_
%{
    #include <assert.h>
    #include <math.h> // floor, ceil
    using djnnstl::cerr;
    using djnnstl::cout;
    using djnnstl::endl;

    #include "gui/picking/picking.h"
    

static
inline
double
get_double_value (const CoreProcess* property)
{
    auto * p = dynamic_cast<const AbstractProperty*>(property);
    assert(p);
    //std::cerr << floor(t->get_double_value()) << std::endl;
    return p->get_double_value();
}

static
inline
uint32_t
get_pixel_color (CoreProcess* frame, int x, int y)
{
    auto * f = dynamic_cast<Window*>(frame);
    assert(f);
    auto res = f->get_pixel_color(x, y);
    puts(to_string(res).c_str());
    return res & 0xffffff;
}

static
inline
AbstractGObj*
pick_graphical_object(CoreProcess* frame, int x, int y)
{
    auto * f = dynamic_cast<Window*>(frame);
    assert(f);
    Picking * picking = f->picking_view();
    assert(picking);
    auto * picked = picking->pick(x, y);
    if (picked != nullptr) {
        // cout << "picked at " << x << " " << y << " = " << picked << endl;
        string hierarchy_name = get_hierarchy_name (dynamic_cast<CoreProcess*> (picked));
        cout << "picked at " << x << " " << y << " = " << hierarchy_name << endl;
    }
    else {
        cout << "Nothing to pick at " << x << " " << y << endl;
    }
    // puts(to_string(picked).c_str());
    return dynamic_cast<AbstractGObj*>(picked);
}

%}

_main_
Component root {
    Frame f("Tests for picking", 200, 200, 800, 800)                    // [7GUIs] The task is to build a frame containing...
    f.close ->! mainloop

    TextPrinter tp

    // Red rectangles
    Component c1 {
        FillColor f1 (#FF0000)

        Rectangle r11 (0, 0, 100, 50)
        r11.press -> {
            "r11" =: tp.input
        }

        Translation t1 (200, 200)

        Rectangle r12 (0, 0, 100, 50)
        r12.press -> {
            "r12" =: tp.input
        }
    }

    // Green rectangles
    Component c2 {
        FillColor f2 (#00FF00)

        Rectangle r21 (350, 20, 100, 50)
        r21.press -> {
            "r21" =: tp.input
        }

        Translation t2 (200, 200)

        Rectangle r22 (350, 20, 100, 50)
        r22.press -> {
            "r22" =: tp.input
        }
    }

    // Blue rectangles
    Component c3 {
        FillColor f3 (#0000FF)

        Rectangle r31 (20, 350, 100, 50)
        r31.press -> {
            "r31" =: tp.input
        }

        Translation t3 (200, 200)

        Rectangle r32 (20, 350, 100, 50)
        r32.press -> {
            "r32" =: tp.input
        }
    }

    // Cyan circles
    Component c4 {
        FillColor f4 (#00FFFF)

        Circle c41 (140, 140, 20)
        c41.press -> {
            "c41" =: tp.input
        }

        Translation t4 (200, 200)

        Circle c42 (140, 140, 20)
        c42.press -> {
            "c42" =: tp.input
        }
    }


    mainloop -> (root) {
        print ("PICKING...")

        // Picking on rect r11
        assert (pick_graphical_object(root.f, 0, 0) == &root.c1.r11)
        assert (pick_graphical_object(root.f, 99, 49) == &root.c1.r11)
        pick_graphical_object(root.f, 100, 50)

        // Picking on rect r21 & r31
        pick_graphical_object(root.f, 351, 21)
        pick_graphical_object(root.f, 21, 351)

        // Picking on rect r12 & r22
        pick_graphical_object(root.f, 201, 201)
        pick_graphical_object(root.f, 551, 221)

        // Picking on rect r32
        pick_graphical_object(root.f, 220, 550)
        pick_graphical_object(root.f, 319, 599)
        pick_graphical_object(root.f, 320, 600)

        // Picking on circle c41 (& outside)
        pick_graphical_object(root.f, 150, 150)
        pick_graphical_object(root.f, 140, 140)
        pick_graphical_object(root.f, 160, 160)
        pick_graphical_object(root.f, 130, 130)
        pick_graphical_object(root.f, 170, 170)

        // Picking on circle c42
        pick_graphical_object(root.f, 340, 340)


        print ("Pixel COLOR...")
        // Test pixel color on r11
        assert (get_pixel_color(root.f, 0, 0) == 0xFF0000)

        // Test pixel color on 21
        assert (get_pixel_color(root.f, 350, 20) == 0x00FF00)

        // Test pixel color on 31
        assert (get_pixel_color(root.f, 20, 350) == 0x0000FF)

        print ("END")
    }



}

