#pragma once

#include "ads/ccpp/interfaces/swath-and-region-producer-if.h"

namespace ads
{
namespace ccpp
{
namespace swath_and_region_producer
{

class SwathAndRegionProducer : public interfaces::SwathAndRegionProducerIf
{
    const double m_swathWidth;

  public:
    SwathAndRegionProducer(double swathWidth);
    ~SwathAndRegionProducer() override = default;

    std::vector<std::pair<geometry::Ring2d, geometry::MultiLine2d>>
    produceSwathsAndRegions(const DoublyConnectedEdgeList& dcel,
                            const std::vector<interfaces::region_merger::MergeRegionGroup>& mergeGroups) override;
};
}

typedef swath_and_region_producer::SwathAndRegionProducer SwathAndRegionProducer;
}
}
