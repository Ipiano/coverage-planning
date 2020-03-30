#include "swath-and-region-producer.h"

#include <unordered_set>

namespace ads
{
namespace ccpp
{
namespace swath_and_region_producer
{

using namespace interfaces::region_merger;
using namespace std;
using namespace geometry;
namespace bg = boost::geometry;

Ring2d makeRegion(const MergeRegionGroup& regionGroup);
MultiLine2d makeSwaths(const Ring2d& region, const MergeRegionGroup& regionGroup, double swathWidth);

SwathAndRegionProducer::SwathAndRegionProducer(const double swathWidth) : m_swathWidth(swathWidth)
{
}

vector<pair<Ring2d, MultiLine2d>> SwathAndRegionProducer::produceSwathsAndRegions(const DoublyConnectedEdgeList&,
                                                                                  const vector<MergeRegionGroup>& mergeGroups)
{
    vector<pair<Ring2d, MultiLine2d>> result;

    for (const auto& mergeGroup : mergeGroups)
    {
        auto region = makeRegion(mergeGroup);
        auto swaths = makeSwaths(region, mergeGroup, m_swathWidth);

        result.emplace_back(move(region), move(swaths));
    }

    return result;
}

Ring2d makeRegion(const MergeRegionGroup& regionGroup)
{
    Ring2d ring;

    std::unordered_set<dcel::region_t*> dcelRegions;
    for (const auto& mergeRegion : regionGroup.regionsToMerge)
        dcelRegions.insert(mergeRegion.dcelRegion);

    // Find an edge that's not shared
    auto firstEdge = regionGroup.regionsToMerge[0].dcelRegion->edge;
    while (firstEdge->twin != nullptr && dcelRegions.count(firstEdge->twin->region) > 0)
    {
        firstEdge = firstEdge->next;
    }

    // Walk around the edge; when a shared edge is found
    // hop over it if it connects to one of the regions in
    // the group
    auto currEdge = firstEdge;
    do
    {
        ring.push_back(currEdge->origin->location);

        currEdge = currEdge->next;

        while (currEdge->twin != nullptr && dcelRegions.count(currEdge->twin->region))
            currEdge = currEdge->twin->next;

    } while (currEdge != firstEdge);

    bg::correct(ring);
    return ring;
}

static quantity::Radians normal(const quantity::Radians angle)
{
    const static auto quarter = static_cast<quantity::Radians>(units::Degree * 90);

    // Keep result in [0, PI)
    return angle >= quarter ? angle - quarter : angle + quarter;
}

MultiLine2d makeSwaths(const Ring2d& region, const MergeRegionGroup& mergeGroup, const double swathWidth)
{
    const static auto quarter = static_cast<quantity::Radians>(units::Degree * 90);

    const auto bbox = bg::return_envelope<Box2d>(region);

    // Max possible length of a swath; +1 to guarantee the ends lie outside the bbox
    const double swathLength = bg::distance(bbox.min_corner(), bbox.max_corner()) + 1;
    const size_t swathCount  = size_t((swathLength / swathWidth) + 0.5);

    MultiLine2d result;
    std::vector<std::vector<Segment2d>> lineGroups;

    // Make swaths covering the region for each sub-region
    for (const auto& subregion : mergeGroup.regionsToMerge)
    {
        Ring2d subregionRing;
        const auto firstEdge = subregion.dcelRegion->edge;
        auto currEdge        = firstEdge;
        do
        {
            subregionRing.push_back(currEdge->origin->location);
            currEdge = currEdge->next;
        } while (currEdge != firstEdge);
        subregionRing.push_back(subregionRing.front());

        bg::correct(subregionRing);
        if (!bg::is_valid(subregionRing))
            continue;

        const auto normalDir = normal(subregion.swathDir);
        const Point2d offset = {cos(normalDir.value()) * swathWidth, sin(normalDir.value()) * swathWidth};

        auto startPoint  = normalDir > quarter ? Point2d(bbox.max_corner().x(), bbox.min_corner().y()) : bbox.min_corner();
        Point2d swathEnd = {cos(subregion.swathDir.value()) * swathLength, sin(subregion.swathDir.value()) * swathLength};
        auto swathLeft = startPoint, swathRight = startPoint;

        bg::subtract_point(swathLeft, swathEnd);
        bg::add_point(swathRight, swathEnd);

        MultiLine2d lines;

        for (size_t i = 0; i < swathCount; i++)
        {
            lines.push_back({swathLeft, swathRight});
            bg::add_point(swathLeft, offset);
            bg::add_point(swathRight, offset);
        }

        MultiLine2d intersectionLines;
        bg::intersection(subregionRing, lines, intersectionLines);

        // TODO: Figure out why some lines are not trimmed at all
        intersectionLines.erase(
            std::remove_if(intersectionLines.begin(), intersectionLines.end(),
                           [&](const Line2d& line) { return bg::disjoint(bbox, line.front()) || bg::disjoint(bbox, line.back()); }),
            intersectionLines.end());

        result.insert(result.end(), intersectionLines.begin(), intersectionLines.end());
    }

    // Find intersections between the ones for adjacent regions, and shrink them to those points
    return result;
}

}
}
}
