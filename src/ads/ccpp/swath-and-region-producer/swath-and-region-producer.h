#pragma once

#include "ads/ccpp/interfaces/swath-and-region-producer-if.h"

namespace ads
{
namespace ccpp
{
namespace swath_and_region_producer
{

/*!
 * \brief Converts a merge region merger solution into a set of loops and swaths
 */
class SwathAndRegionProducer : public interfaces::SwathAndRegionProducerIf
{
    const double m_swathWidth;

  public:
    /*!
     * \brief Inits the SwathAndRegionProduer with the target width between swaths
     * \param swathWidth Width between swaths. This should use whatever units the original polygon data was in
     */
    SwathAndRegionProducer(double swathWidth);
    ~SwathAndRegionProducer() override = default;

    /*!
     * \brief Consumes a RegionMergerIf solution and produces the set of distinct loops and swaths inside of them indicated
     *
     * Swaths are produced by finding a bounding box for the entire input shape and stepping from one corner to the other,
     * placing lines across the box in the target direction. Those lines are then truncated wherever they intersect the
     * boundary. This will likely not generate the most efficient set of swaths; doing so would require taking a bit
     * extra care to place the first swath as close as possible to one edge of the shape.
     *
     * \param dcel Doubly-Connected Edge List containing the base regions of the shape
     * \param mergeGroups List of groups of regions that should be merged with their final swath directions
     * \return List of loops and the associated multi-line of swaths that are inside that loop
     */
    std::vector<std::pair<geometry::Ring2d, geometry::MultiLine2d>>
    produceSwathsAndRegions(const Dcel& dcel, const std::vector<interfaces::region_merger::MergeRegionGroup>& mergeGroups) override;
};
}

typedef swath_and_region_producer::SwathAndRegionProducer SwathAndRegionProducer;
}
}
