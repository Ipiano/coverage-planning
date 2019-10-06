#pragma once

#include "decomposition.h"

#include <boost/geometry.hpp>

namespace ccpp {

template<class BoostPolygonType>
class SwathPlanner
{
    typedef PolygonDecomposer<BoostPolygonType> DecomposerType;
public:
    SwathPlanner(DecomposerType*)
    {}
};

}
