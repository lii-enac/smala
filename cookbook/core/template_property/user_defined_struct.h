#pragma once

// example with user-defined type

struct MyPoint2D {
  //MyPoint2D() : x(x_), y(y_) {}
  double x,y;
};

extern MyPoint2D pdef;

inline
const MyPoint2D
operator+(const MyPoint2D& a, const MyPoint2D& b)
{
  return MyPoint2D{a.x+b.x,a.y+b.y};
}

// inline
// ostream&
// operator<<(ostream& out, const MyPoint2D& a)
// {
//   out << a.x << " " << a.y;
//   return out;
// }

// example with units

#include "units.h"
using namespace units::literals;
namespace units {
  UNIT_ADD(length, flightLevel, flightLevels, fl, unit<std::ratio<100>, feet>) // add flight level as a unit
}

using namespace units::length;
using namespace units::time;
using namespace units::velocity;

inline std::string to_string2 (const meters_per_second_t s) { return to_string(s); }


// example with arrray of doubles
//#include <execution>
//#include <algorithm>
#include <valarray> // valarray has all math operator defined...

using vector_double = std::valarray<double>; //vector<double>;
using DoubleArray = TemplateProperty<vector_double>;
extern vector_double vd_def;

/*inline
const vector_double&
operator+(const vector_double& a, const vector_double& b)
{
  //vector_double res(a.size());
  //std::transform(std::execution::par_unseq, begin(a), end(a), begin(b), begin(res), operator+);
  return a+b;
}*/