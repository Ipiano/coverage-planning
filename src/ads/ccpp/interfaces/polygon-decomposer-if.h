#pragma once

#include "ads/ccpp/typedefs.h"
#include "ads/ccpp/dcel.h"

#include <vector>

namespace ads
{
namespace ccpp
{
namespace interfaces
{

class PolygonDecomposerIf
{
  public:
    virtual ~PolygonDecomposerIf() = default;

    /*!
     * \brief Decomposes a polygon into approximately trapezoidal regions
     * \param[in] poly - Polygon to decompose
     *
     * \returns A DoublyConnectedEdgeList which represents all the regions produced
     *
     * \throws std::invalid_argument if the given polygon is malformed
     */
    virtual DoublyConnectedEdgeList decomposePolygon(const geometry::Polygon2d& poly) const = 0;
};
}
}
}
