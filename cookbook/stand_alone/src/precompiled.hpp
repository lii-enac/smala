//#include <iostream>
//#include "core/utils/build/precompiled.hpp"

#include "core/utils/containers.h"
#include "core/ontology/process.h"

namespace djnn {
    class CoreProcess;
    class Coupling;
}

template class djnnstl::basic_string<char>;
template class djnn::map<djnn::string, djnn::CoreProcess*>;
template class djnn::vector<djnn::Coupling*>;