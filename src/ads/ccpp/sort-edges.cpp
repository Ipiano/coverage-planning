#include "sort-edges.h"

#include <utility>
#include <tuple>

namespace bg = boost::geometry;

namespace ads
{
namespace ccpp
{
typedef std::pair<geometry::Point2d, const dcel::const_half_edge_t*> SortEdge;

std::pair<std::vector<SortEdge>, geometry::Point2d> makeEdgeListAndFindCentroid(const std::vector<const dcel::const_half_edge_t*>& edges)
{
    std::vector<SortEdge> segmentsWithEdges(edges.size());

    // Make list of all segments with the edge it's attached to
    // Also find a centroid for the shape
    auto centroid = bg::make_zero<geometry::Point2d>();
    std::transform(edges.begin(), edges.end(), segmentsWithEdges.begin(),
                   [&](const dcel::const_half_edge_t* edge)
                   {
                       bg::add_point(centroid, edge->origin->location);
                       return std::make_pair(edge->origin->location, edge);
                   });

    bg::divide_value(centroid, edges.size());

    return std::make_pair(std::move(segmentsWithEdges), centroid);
}

void sortEdgeList(std::vector<SortEdge>& segmentsWithEdges, const geometry::Point2d &centroid, const quantity::Radians &sweepDir)
{
    // Move segments so origin is the centroid
    // Then rotate around the centroid by -sweep angle
    // So that the sweep angle is the positive X direction
    // Note: Sweep angle is defined counter-clockwise, but boost rotation is clockwise
    //      so the negative disappears
    bg::strategy::transform::rotate_transformer<bg::radian, double, 2, 2> rotate(sweepDir.value());
    bg::strategy::transform::translate_transformer<double, 2, 2> translate(-bg::get<0>(centroid), -bg::get<1>(centroid));

    std::transform(segmentsWithEdges.begin(), segmentsWithEdges.end(), segmentsWithEdges.begin(),
                   [rotate, translate](const SortEdge& edge){
                       geometry::Point2d translated, rotated;

                       bg::transform(edge.first, translated, translate);
                       bg::transform(translated, rotated, rotate);

                       return SortEdge(rotated, edge.second);
                   });

    // Sort edges left-right, bottom-top WRT the new origin and rotation
    std::sort(segmentsWithEdges.begin(), segmentsWithEdges.end(),
              [](const SortEdge& l, const SortEdge& r) -> bool
              {
                  const auto& pl = l.first;
                  const auto& pr = r.first;

                  return std::abs(bg::get<0>(pl) - bg::get<0>(pr)) < 1e-10 ?
                                  bg::get<1>(pl) < bg::get<1>(pr) : bg::get<0>(pl) < bg::get<0>(pr);
              });
}

void sortEdges(std::vector<const dcel::const_half_edge_t*>& edges, const quantity::Radians sweepDir)
{
    auto edgeListAndCentroid = makeEdgeListAndFindCentroid(edges);

    sortEdgeList(edgeListAndCentroid.first, edgeListAndCentroid.second, sweepDir);

    std::transform(edgeListAndCentroid.first.begin(), edgeListAndCentroid.first.end(), edges.begin(),
                   [](const SortEdge& edge)
                   {
                       return edge.second;
                   });

}

}
}
