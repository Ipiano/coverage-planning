#include "active-edges-list.h"

#include "ads/assertion.h"
#include "ads/epsilon.h"

namespace bg = boost::geometry;

namespace ads
{
namespace algorithms
{
namespace sweep_line
{

template <class T> std::pair<T, T> minMax(const T& a, const T& b)
{
    return a < b ? std::make_pair(a, b) : std::make_pair(b, a);
}

/*!
 * \brief Gets any intersection between two edges
 *
 * It is expected that this is only called during the active edge less than
 * comparison, where there should always be at least one. In case that's an
 * invalid assertion, or there are edge cases that boost does not quite handle
 * as we want, this uses the ASSERT macro defined in assertion.h to throw an
 * AssertionFailure when there are no intersections.
 *
 * \param s1 First segment
 * \param s2 Second segment
 * \return Any intersection point on the segments
 */
Point2d anyIntersection(const bg::model::referring_segment<const Point2d>& s1, const bg::model::segment<Point2d>& s2)
{
    bg::model::multi_point<Point2d> intersections;
    bg::intersection(s1, s2, intersections);
    ASSERT(intersections.size() > 0);

    return intersections[0];
}

// For the sake of the active edge list, we keep track of them 'vertically'
// by checking if a point on the second is to the left of a point on the other
// line. If this is the case then we know the second one must be 'above' the first
// one for its entire distance because there are now intersections
struct ActiveEdgeLessThan
{
    bool operator()(const PolygonEdge* l, const PolygonEdge* r)
    {
        const auto l1 = l->firstPoint();
        const auto l2 = l->secondPoint();

        const auto r1 = r->firstPoint();
        const auto r2 = r->secondPoint();

        // First check if one is just wholly above the other
        double minYL, maxYL;
        std::tie(minYL, maxYL) = minMax(l1.y(), l2.y());

        double minYR, maxYR;
        std::tie(minYR, maxYR) = minMax(r1.y(), r2.y());

        if (maxYL < minYR)
            return true;
        else if (maxYR < minYL)
            return false;

        // If not the trivial case, then find the stretch of the X axis that both lines
        // are in; in theory, this should always exist because of how the sweep line algo
        // works
        //
        // Once the min and max X coordinates that both lines share are found, we can check
        // the segments along that stretch to see if one is under the other

        double minXL, maxXL;
        std::tie(minXL, maxXL) = minMax(l1.x(), l2.x());

        double minXR, maxXR;
        std::tie(minXR, maxXR) = minMax(r1.x(), r2.x());

        const auto sharedRangeStart = std::max(minXL, minXR);
        const auto sharedRangeEnd   = std::min(maxXL, maxXR);

        const auto minY = std::min(minYR, minYL) - 1;
        const auto maxY = std::max(maxYR, maxYL) + 1;

        ASSERT(sharedRangeStart <= sharedRangeEnd);

        bg::model::segment<Point2d> vertical{{sharedRangeStart, minY}, {sharedRangeStart, maxY}};

        const auto p1L = anyIntersection({l1, l2}, vertical);
        const auto p1R = anyIntersection({r1, r2}, vertical);

        // If the first points are the same, check the second; otherwise just check if one
        // is under the other
        if (std::abs(p1L.y() - p1R.y()) < epsilon)
        {
            vertical.first.set<0>(sharedRangeEnd);
            vertical.second.set<0>(sharedRangeEnd);

            const auto p2L = anyIntersection({l1, l2}, vertical);
            const auto p2R = anyIntersection({r1, r2}, vertical);

            return p2L.y() < p2R.y();
        }
        return p1L.y() < p1R.y();
    }
};

long ActivePolygonEdgesList::size() const
{
    return static_cast<long>(m_edges.size());
}

long ActivePolygonEdgesList::insert(PolygonEdge* e)
{
    const auto currEdgeInsertIt = std::lower_bound(m_edges.begin(), m_edges.end(), e, ActiveEdgeLessThan());
    const auto activeEdgeIndex  = std::distance(m_edges.begin(), currEdgeInsertIt);

    // Prevent double insertions
    if (currEdgeInsertIt == m_edges.end() || *currEdgeInsertIt != e)
        m_edges.insert(currEdgeInsertIt, e);

    return activeEdgeIndex;
}

void ActivePolygonEdgesList::removeLessThanEqual(const PolygonEdge* e)
{
    // Remove edges that are strictly to the left
    removeLessThan(e);

    // Remove the edge attached to this one
    // that is 'left' of it
    removeEdgeToLeft(e);
}

void ActivePolygonEdgesList::removeEdgeToLeft(const PolygonEdge* e)
{
    // Removes the edge attached to the 'first' point of e, except
    // for two cases - When e is a zig-zag, and the left point
    // is connected to the top edge of e; and when e is one of the
    // left-most edges on a loop. In the first case, e->secondEdge is
    // removed; in the latter, nothing is removed

    // Special Case 1:
    // Check if vertical zig zag
    if (e->isVerticalZigZag() && equal(e->secondEdge()->secondPoint(), e->secondPoint()))
        removeSpecific(e->secondEdge());

    // Common Case
    // Check if previous is zig zag or previous second matches first
    else if (e->firstEdge()->isVerticalZigZag() || equal(e->firstEdge()->secondPoint(), e->firstPoint()))
        removeSpecific(e->firstEdge());
}

void ActivePolygonEdgesList::removeLessThan(const PolygonEdge* e)
{
    removeLessThan(e->firstPoint());
}

void ActivePolygonEdgesList::removeLessThan(const Point2d& p)
{
    m_edges.erase(std::remove_if(m_edges.begin(), m_edges.end(), [&](const PolygonEdge* e) { return (e->secondPoint().x() < p.x()); }),
                  m_edges.end());
}

void ActivePolygonEdgesList::removeSpecific(const PolygonEdge* e)
{
    const auto targetIt = std::find(m_edges.begin(), m_edges.end(), e);
    if (targetIt != m_edges.end())
        m_edges.erase(targetIt);
}

// Returns the index of the given edge if it is present, otherwise
// -1
long ActivePolygonEdgesList::indexOf(const PolygonEdge* e) const
{
    const auto edgeIt = std::lower_bound(m_edges.begin(), m_edges.end(), e, ActiveEdgeLessThan());
    if (edgeIt != m_edges.end() && *edgeIt == e)
        return std::distance(m_edges.begin(), edgeIt);
    return -1;
}

// Returns the index of the edge which is (or would be) below
// the given edge. If the given edge is or would be the first edge
// in the list, -1 is returned
long ActivePolygonEdgesList::indexBelow(const PolygonEdge* e) const
{
    const auto edgeIt = std::lower_bound(m_edges.begin(), m_edges.end(), e, ActiveEdgeLessThan());
    return std::distance(m_edges.begin(), std::prev(edgeIt));
}

PolygonEdge* ActivePolygonEdgesList::operator[](const long index) const
{
    return m_edges[static_cast<size_t>(index)];
}
}
}
}
