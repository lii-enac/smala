/*
 * Adapted from the code explained here: https://dragonman225.js.org/curved-arrows.html
 */

use core
use base
use gui

_native_code_ 
%{
#include <iostream>
#include <vector>
#include <cmath>

static double start_x = 0;
static double start_y = 0;
static double end_x = 0;
static double end_y = 0;
static double control_start_x = 0;
static double control_start_y = 0;
static double control_end_x = 0;
static double control_end_y = 0;

static int angle = 0;
static int arrow_size = 5;

static double get_start_x () { return start_x; }
static double get_start_y () { return start_y; }
static double get_end_x () { return end_x; }
static double get_end_y () { return end_y; }
static double get_control_start_x () { return control_start_x; }
static double get_control_start_y () { return control_start_y; }
static double get_control_end_x () { return control_end_x; }
static double get_control_end_y () { return control_end_y; }
static int get_angle () { return angle; }
static int get_arrow_size () { return arrow_size; }
enum sides {top, right, bottom, left};


static double
distance (pair <double, double> p1, pair <double, double> p2)
{
    return sqrt(pow(p1.first - p2.first, 2) + pow (p1.second - p2.second, 2));
}

static bool
is_in_box (pair <double, double> point, double box[])
{
    return (point.first > box[0] &&
            point.first < (box[0] + box[2]) &&
            point.second > box[1] &&
            point.second < (box[1] + box[3]));
}

static pair<double, double>
compute_control_point (pair<double,double> target, pair<double, double>from, sides side)
{
    int min_distance_to_target = 50;
    pair<double,double> point;
    switch (side) {
        case top: {
            point.first = target.first;
            point.second = fmin((target.second + from.second) / 2, target.second - min_distance_to_target);
            return point;
        }
        case bottom: {
            point.first = target.first;
            point.second = fmax((target.second + from.second) / 2, target.second + min_distance_to_target);
            return point;
        }
        case left: {
            point.first = fmin((target.first + from.first) / 2, target.second - min_distance_to_target);
            point.second = target.second;
            return point;
        }
        case right: {
            point.first = fmax((target.first + from.first) / 2, target.first + min_distance_to_target);
            point.second = target.second;
            return point;
        }
    }
}

 /*
 * 0 - top, 1 - right, 2 - bottom, 3 - left
 */
static void
compute_shortest_distance (int x1, int y1, int w1, int h1, int x2, int y2, int w2, int h2)
{
    double shortest = -1;
    vector< pair<double, double> > starting_points = { {x1+w1/2, y1}, // top
                                                       {x1+w1, y1+h1/2}, // right
                                                       {x1+w1/2, y1+h1}, // bottom
                                                       {x1, y1+h1/2} // left
                                                     };
    vector< pair<double, double> > ending_points = { {x2+w2/2, y2 - 2*arrow_size}, // top
                                                     {x2+w2+2*arrow_size, y2+h2/2}, // right
                                                     {x2+w2/2, y2+h2+2*arrow_size}, // bottom
                                                     {x2-2*arrow_size, y2+h2/2} // left
                                                    };
    double start_box[4] = {x1 - 15.0, y1 - 15.0, w1 + 30.0, h1 + 30.0}; // adding an exclusion zone
    double end_box[4] = {x2 - 15.0, y2 - 15.0, w2 + 30.0, h2 + 30.0};  // adding an exclusion zone
    int idx_end, idx_start, cur_idx_start, cur_idx_end;
    idx_end = idx_start = cur_idx_start = cur_idx_end = 0;
    for (auto start_point: starting_points) {
        if (is_in_box (start_point, end_box)) continue;
        cur_idx_end = 0;    
        for (auto end_point : ending_points) {
            if (is_in_box (end_point, start_box)) continue;
            double d = distance (start_point, end_point);
            if (shortest == -1 || d < shortest) {
                start_x = start_point.first;
                start_y = start_point.second;
                end_x = end_point.first;
                end_y = end_point.second;
                shortest = d;
                idx_start = cur_idx_start;
                idx_end = cur_idx_end;                
            }
            cur_idx_end++;
        }
        cur_idx_start++;
    }
    pair<double, double> point = compute_control_point (starting_points[idx_start], ending_points[idx_end], (sides)idx_start);
    control_start_x = point.first;
    control_start_y = point.second;
    point = compute_control_point (ending_points[idx_end], starting_points[idx_start], (sides)idx_end);
    control_end_x = point.first;
    control_end_y = point.second;
    switch (idx_end) {
        case top:
            angle = 90;
            break;
        case right:
            angle = 180;
            break;
        case bottom:
            angle = 270;
            break;
        case left:
            angle = 0;
            break;
        default:;
    }
}

static void
set_double (Process *p, double v) {
    AbstractDoubleProperty *dp = dynamic_cast<AbstractDoubleProperty*> (p);
    if (dp) {
        dp->set_value (v, true);
    } else {
        std::cout << "fail to cast double property\n";
    }
}

%}

_define_
Arrow (Process _src) {
    int arrow_size = get_arrow_size()
    OutlineWidth _ (2)
    OutlineColor _ (#AFAFAF)

    RefProperty dst (null)
    RefProperty src (_src)
    
    Double angle (0)
    FillColor _ (#AFAFAF)
    Circle start_circle (_src.x + _src.width/2, $_src.y, 5)
    NoFill _
    Path path {
        //PathMove _ (src.x + src.width/2, $src.y)
        PathMove _ ($GenericMouse.x, $GenericMouse.y)
        PathCubic _ ($GenericMouse.x, 0, 0, $GenericMouse.y, 5, 5)
    }

    path.items.[1].{x,y} =:> start_circle.{cx,cy}

    dst->update_path:(this) {
        dst = getRef(this.dst)
        src = getRef(this.src)
        compute_shortest_distance ($src.x, $src.y, $src.width, $src.height, $dst.x, $dst.y, $dst.width, $dst.height)
        set_double (this.path.items.[1].x, get_start_x ())
        set_double (this.path.items.[1].y, get_start_y ())
        set_double (this.path.items.[2].x1, get_control_start_x ())
        set_double (this.path.items.[2].x2, get_control_end_x ())
        set_double (this.path.items.[2].y1, get_control_start_y ())
        set_double (this.path.items.[2].y2, get_control_end_y ())
        set_double (this.path.items.[2].y, get_end_y())
        set_double (this.path.items.[2].x, get_end_x())
        set_double (this.angle, get_angle ())
    }
    dst->set_up_bindings:(this) {
        dst = getRef(this.dst)
        src = getRef(this.src)
        addChildrenTo this {
            src.x -> this.update_path
            src.y -> this.update_path
            src.width -> this.update_path
            src.height -> this.update_path

            dst.x -> this.update_path
            dst.y -> this.update_path
            dst.width -> this.update_path
            dst.height -> this.update_path
        }
    }
    FSM fsm {
        State init {
            (GenericMouse.x + _src.x)/2=:> path.items.[2].x1, path.items.[2].x2
            _src.y =:> path.items.[2].y1
            GenericMouse.y =:> path.items.[2].y2, path.items.[2].y
            GenericMouse.x =:> path.items.[2].x

        }
        State two_boxes {
            FillColor _ (#AFAFAF)
            Translation pos_arrow (0, 0)
            Rotation rot (5, 0, 0)
            Polygon arrow_end {
                Point _ (0, -arrow_size)
                Point _ (2*arrow_size,0)
                Point _ (0, arrow_size)
            }
            angle =:> rot.a
            path.items.[2].{x,y} =:> pos_arrow.{tx,ty}
        }
        init->two_boxes (dst)
    }

}