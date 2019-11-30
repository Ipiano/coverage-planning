#pragma once

#include "ads/ccpp/decomposition/polygon_decomposer_if.h"

#include <boost/geometry.hpp>

namespace ads {
namespace ccpp {

template <class BoostPolygonType>
class SwathPlanner
{
    typedef PolygonDecomposer<BoostPolygonType> DecomposerType;

    public:
        SwathPlanner(DecomposerType*) {}
};

}  // namespace ccpp
}  // namespace ads
