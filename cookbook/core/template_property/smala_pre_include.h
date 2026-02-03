#include "core/utils/containers/string.h"
#include "core/utils/to_string.h"
using djnnstl::string;
using djnnstl::to_string;

#include "user_defined_struct.h"

inline
std::ostream&
operator<<(std::ostream& out, const MyPoint2D& arg)
{
  out << to_string2(arg);
  return out;
}

inline
std::ostream&
operator<<(std::ostream& out, const vector_double& arg)
{
  out << to_string2(arg);
  return out;
}
