#pragma once

#include "polygon_decomposer_if.h"

#include <boost/geometry.hpp>

namespace ccpp {

template<class BoostPolygonType>
class SwathPlanner
{
  typedef PolygonDecomposer<BoostPolygonType> DecomposerType;

public:
  SwathPlanner(DecomposerType*) {}
};
}
