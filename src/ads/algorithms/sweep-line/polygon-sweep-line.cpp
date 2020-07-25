#include "polygon-sweep-line.h"

#include "ads/assertion.h"

#include <boost/geometry/algorithms/for_each.hpp>

namespace bg = boost::geometry;

namespace ads
{
namespace algorithms
{
namespace sweep_line
{

std::vector<std::unique_ptr<PolygonEdge>> PolygonSweepLineAlgorithm::buildSortedEdgeList(const Polygon2d& poly)
{
    typedef std::unique_ptr<PolygonEdge> EdgePtr;
    std::vector<EdgePtr> edges;

    //  a. Build list of edges such that
    //      * Every segment's first and second points are sorted
    //      * Every segment points to the one before and after it when traversing the polygon

    // Put exterior ring first so that it'll be ring index 0
    std::vector<bg::ring_type<Polygon2d>::type> rings{poly.outer()};
    rings.insert(rings.end(), poly.inners().begin(), poly.inners().end());

    for (size_t i = 0; i < rings.size(); i++)
    {
        const auto& ring = rings[i];

        if (ring.size() < 3)
            throw std::invalid_argument("empty loop");

        // Track how many edges were in the list before we started this loop
        // so we can iterate through that group at the end to link them up
        const auto firstEdge = edges.size();

        bg::for_each_segment(ring, [&](const bg::model::referring_segment<const Point2d>& segment) {
            edges.emplace(edges.end(), new PolygonEdge(segment.first, segment.second));
        });

        // Link everything up
        for (auto k = edges.size() - 1, j = firstEdge, i = firstEdge + 1; i < edges.size(); j = i++, k = j - 1)
            edges[j]->setAdjacentEdges(edges[k].get(), edges[i].get(), i == 0);

        edges.back()->setAdjacentEdges((edges.end() - 2)->get(), edges[firstEdge].get(), i == 0);
    }

    //  b. Sort segments
    //      Sort by leftmost point, unless they're the same, then sort by other point. Points being the 'same'
    //      can be checked by seeing if they're the same vertex pointer, becuase we merged points that are physically
    //      close to each other in the last step
    std::sort(edges.begin(), edges.end(), [](const EdgePtr& l, const EdgePtr& r) { return *l < *r; });

    return edges;
}

PolygonSweepLineAlgorithm::PolygonSweepLineAlgorithm() : m_ran(false)
{
}

PolygonSweepLineAlgorithm::~PolygonSweepLineAlgorithm()
{
}

bool PolygonSweepLineAlgorithm::run(const Polygon2d& polygon)
{
    if (m_ran)
        return false;
    m_ran = true;

    ActivePolygonEdgesList activeEdges;
    auto edges = buildSortedEdgeList(polygon);

    sweepBegin(edges.begin(), edges.end());

    auto sweepLineIt = edges.begin();
    while (sweepLineIt != edges.end())
    {
        auto firstEdgeOfStep = sweepLineIt;

        activeEdges.removeLessThan(sweepLineIt->get());

        // Add all other edges that start at the current sweep line location to the
        // active edge list; this is important so that if we need to make a vertical line
        // segment to break the shape here, it doesn't go all the way up to the outer boundary
        // because we haven't processed the line directly above the current one

        // After this loop, there should always
        // be an even number of edges in the active edges list
        while (sweepLineIt != edges.end() && haveSameXCoord((*sweepLineIt)->firstPoint(), (*firstEdgeOfStep)->firstPoint()))
        {
            const auto sweepEdge = sweepLineIt->get();

            // Removes the edge attached to the left side of this edge
            // (if it's not a start of a loop edge), to maintain the
            // 'activeEdges.size() is even' invariant
            activeEdges.removeEdgeToLeft(sweepEdge);
            activeEdges.insert(sweepEdge);

            sweepLineIt++;
        }

        ASSERT(activeEdges.size() % 2 == 0);

        sweepStep(firstEdgeOfStep, sweepLineIt, activeEdges);
    }

    sweepEnd(edges.begin(), edges.end());

    return true;
}
}
}
}
