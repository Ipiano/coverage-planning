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
     * \brief Decomposes a polygon into approximately trapezoidal regions by inserting vertical lines
     *
     * It is important that all splitting of the polygon be done ONLY by insertion of vertical lines.
     * Vertical, in this case, means that the x coordinate of both points on the segment is the same.
     *
     * The returned Doubly-Connected Edge List should have the following properties (in addition to
     * the properties of a standard DCEL)
     * * The only values in the DCEL that might be nullptr are the twin pointers of half edges
     * * The 'outside' area and 'inside hole' areas of the original polygon should NOT be reflected in
     *      the DCEL at all. This means that most of the edges will not have twins
     * * All edges which DO have a twin are edges that were created to split the polygon, and they are vertical
     * * For any region in the DCEL, there is no holes in any region, nor are there concavities on the left or
     *      right side of the region.
     *
     * Regions ARE allowed to be degenerate (just a vertical line).
     *
     * Regions are allowed to be adjacent to more than 4 regions. The original algorithm that this
     * project is based upon did not support that case, but it is allowed in the context of this project.
     *
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
