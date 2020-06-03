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
struct MergeRegion
{
    dcel::Region dcelRegion;
    quantity::Radians swathDir;
};

struct MergeRegionGroup
{
    std::vector<MergeRegion> regionsToMerge;
};
}

class RegionMergerIf
{
  public:
    virtual ~RegionMergerIf() = default;

    /*!
     * \brief Recursively considers pairs of adjacent regions and merges them when appropriate
     *
     * Assumes that the given DCEL follows the rules outlined in the description of
     * PolygonDecomposerIf
     *
     * \param[in] dcel Doubly-Connected Edge List with regions to consider merging
     *
     * \return List of MergeRegionGroups; where each indicates a group of regions that should
     * be merged together. All regions in the input must be in exactly one of the output MergeRegionGroups.
     * Regions in a group should be ordered from left to right
     */
    virtual std::vector<region_merger::MergeRegionGroup> mergeRegions(const Dcel& dcel) = 0;
};
}
}
}
