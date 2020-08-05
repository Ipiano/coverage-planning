#pragma once

#include "ads/ccpp/interfaces/region-merger-if.h"
#include "ads/ccpp/interfaces/optimal-direction-calculator-if.h"

namespace ads
{
namespace ccpp
{
namespace region_merger
{

/*!
 * \brief Merges regions of a decomposed polygon according to the QuickOpp algorithm merge step
 */
class RegionMerger : public interfaces::RegionMergerIf
{
  public:
    /*!
     * \brief Initializes the merger with a cost calculator
     * \param dirCalculator Cost calculator to be used to determine if a merge is advantageous or not
     */
    RegionMerger(const interfaces::OptimalDirectionCalculatorIf& dirCalculator);
    ~RegionMerger() override = default;

    /*!
     * \brief Merges regions according to the QuickOpp algorithm
     *
     * This performs a depth-first-search iteration through the regions, stepping
     * from one region to the regions that share edges with it in order to evaluate
     * possible region pairs to be merged.
     *
     * For each region pair, the regions are merged if it is a lower cost to use
     * either region's original direction, the direction perpendicular to either region,
     * or both regions' original directions with a bend in on the shared edge.
     *
     * \param dcel Doubly connected edge list specifying the regions to consider merging
     * \return List of MergeRegionGroups each containing a list of regions that should be merged and their final swathing directions
     */
    std::vector<interfaces::region_merger::MergeRegionGroup> mergeRegions(const Dcel& dcel) override;

  private:
    const interfaces::OptimalDirectionCalculatorIf& m_dirCalculator;
};
}

typedef region_merger::RegionMerger RegionMerger;
}
}
