#pragma once

#include "ads/ccpp/typedefs.h"
#include "ads/ccpp/interfaces/region-merger-if.h"
#include "ads/dcel/dcel.h"

namespace ads
{
namespace ccpp
{
namespace interfaces
{

/*!
 * \brief Interface for the final step of the ccpp algorithm which produces the final regions and swaths as output
 */
class SwathAndRegionProducerIf
{
  public:
    virtual ~SwathAndRegionProducerIf() = default;

    /*!
     * \brief Takes a decomposed polygon and a list of regions to merge, and produces the merged regions and swaths for them
     * \param dcel DCEL of the decomposed polygon
     * \param mergeGroups Groupings of regions to merge together and the directions for swathing them
     * \return List of pairs of loop + swaths
     */
    virtual std::vector<std::pair<geometry::Ring2d, geometry::MultiLine2d>>
    produceSwathsAndRegions(const Dcel& dcel, const std::vector<region_merger::MergeRegionGroup>& mergeGroups) = 0;
};
}
}
}
