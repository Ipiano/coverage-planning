#pragma once

#include "ads/dcel/dcel.h"

namespace ads
{
namespace ccpp
{
namespace interfaces
{
namespace region_merger
{

/*!
 * \brief Specification of a dcel::Region and the direction it should be swathed in a RegionMergerIf solution
 */
struct MergeRegion
{
    dcel::Region dcelRegion;
    quantity::Radians swathDir;
};

/*!
 * \brief Grouping of regions in the solution produced by a RegionMergerIf
 *
 * All regions in a group may not have the same direction for swathing; however, their
 * being in the same group should indicate that the solution is to travel from one to the
 * next by turning on the shared edge, rather than complete each of the regions separately.
 */
struct MergeRegionGroup
{
    std::vector<MergeRegion> regionsToMerge;
};
}

/*!
 * \brief Interface for the region-merging step of the ccpp algorithm
 */
class RegionMergerIf
{
  public:
    virtual ~RegionMergerIf() = default;

    /*!
     * \brief Combines regions in a DCEL and optionally defines new directions for them in order to produce a lower-cost solution
     *
     * Assumes that the given DCEL follows the rules outlined in the description of
     * PolygonDecomposerIf
     *
     * \param[in] dcel Doubly-Connected Edge List with regions to consider merging
     *
     * \return List of MergeRegionGroups and total cost using this solution. Each group indicates a group of
     * regions that should be merged together. All regions in the input must be in exactly one of the output
     * MergeRegionGroups. Regions in a group should be ordered from left to right
     */
    virtual std::pair<std::vector<region_merger::MergeRegionGroup>, double> mergeRegions(const Dcel& dcel) = 0;
};
}
}
}
