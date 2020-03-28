#pragma once

#include "ads/ccpp/interfaces/region-merger-if.h"
#include "ads/ccpp/interfaces/optimal-direction-calculator-if.h"

namespace ads
{
namespace ccpp
{
namespace region_merger
{

class RegionMerger : public interfaces::RegionMergerIf
{
  public:
    RegionMerger(const interfaces::OptimalDirectionCalculatorIf& dirCalculator);
    ~RegionMerger() override = default;

    std::vector<interfaces::region_merger::MergeRegionGroup> mergeRegions(const DoublyConnectedEdgeList& dcel) override;

  private:
    const interfaces::OptimalDirectionCalculatorIf& m_dirCalculator;
};

}

typedef region_merger::RegionMerger RegionMerger;

}
}
