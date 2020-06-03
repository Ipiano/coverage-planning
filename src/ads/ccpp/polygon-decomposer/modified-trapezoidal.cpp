#include "modified-trapezoidal.h"

#include "ads/ccpp/coordinate-transform.hpp"

#include <boost/geometry/strategies/transform.hpp>

#include <algorithm>
#include <vector>
#include <unordered_set>
#include <queue>

namespace bg = boost::geometry;

//! < 1mm, if units are meters
//! \todo Make configurable or an argument to algorithm
constexpr static double EPSILON = 0.00000001;

namespace ads
{
namespace ccpp
{
namespace polygon_decomposer
{

struct AssertionFailure : public std::runtime_error
{
    AssertionFailure(const std::string& str) : std::runtime_error(str) {}
};

#define Q(x) #x
#define QUOTE(x) Q(x)
#define ASSERT(x)                                                                 \
    if (!(x))                                                                     \
    {                                                                             \
        throw AssertionFailure(QUOTE(__FILE__) ":" QUOTE(__LINE__) "-" QUOTE(x)); \
    }

//! < operator for verticies in a DCEL, sorts left->right, bottom->top
struct VertexLessThan
{
    bool operator()(const dcel::Vertex& left, const dcel::Vertex& right) { return operator()(left.point(), right.point()); };
    bool operator()(const dcel::Vertex& left, const geometry::Point2d& right) { return operator()(left.point(), right); };
    bool operator()(const geometry::Point2d& left, const dcel::Vertex& right) { return operator()(left, right.point()); };

    bool operator()(const geometry::Point2d& left, const geometry::Point2d& right)
    {
        if (left.x() < right.x())
            return true;
        else if (left.x() > right.x())
            return false;
        else
            return left.y() < right.y();
    };
};

//! Checks if two DCEL vertices have the same X coord (within reason)
struct VertexXCoordEqual
{
    bool operator()(const dcel::Vertex v1, const dcel::Vertex v2)
    {
        const auto p1 = v1.point();
        const auto p2 = v2.point();

        return std::abs(p1.x() - p2.x()) < EPSILON;
    }
};

template <class SegmentT> geometry::Point2d unit(const SegmentT s)
{
    const double mag = bg::distance(s.first, s.second);

    geometry::Point2d unitV = s.second;
    bg::subtract_point(unitV, s.first);
    bg::divide_value(unitV, mag);

    return unitV;
}

struct Edge
{
    // Pointer to the most recent dcel edge created
    // which ends at the right-most point of this segment.
    // The dcel edge for the edge attached to this one
    // should point to this half edge.
    dcel::HalfEdge halfEdge;

    // Gets the points of the edge connected to 'firstEdge()' and 'secondEdge()' respectively
    dcel::Vertex firstPoint() const { return first_; }
    dcel::Vertex secondPoint() const { return second_; }

    // Gets the points of the edge connected to 'leftEdge()' and 'rightEdge()' respectively
    dcel::Vertex leftPoint() const
    {
        return firstPoint() == leftEdge()->firstPoint() || firstPoint() == leftEdge()->secondPoint() ? firstPoint() : secondPoint();
    }
    dcel::Vertex rightPoint() const
    {
        return firstPoint() == rightEdge()->firstPoint() || firstPoint() == rightEdge()->secondPoint() ? firstPoint() : secondPoint();
    }

    // Gets the edge attached at a specific point on the edge
    Edge* firstEdge() const
    {
        if (nextEdge()->firstPoint() == firstPoint() || nextEdge()->secondPoint() == firstPoint())
            return nextEdge();
        return previousEdge();
    }

    Edge* secondEdge() const { return firstEdge() == nextEdge() ? previousEdge() : nextEdge(); }

    // Gets the edges left or right of the current edge respectively. This is the same as firstEdge()/secondEdge()
    // except for the special case. This means that the start of loop edges will return each other as the 'left' edge
    // and the end of loop edges will return each other as the 'right' edge
    // -->--*
    //      |
    //      ^
    //      |
    //      *-->--
    Edge* leftEdge() const
    {
        return !isVerticalZigZag() ? firstEdge() : (firstEdge()->firstPoint() == firstPoint() ? secondEdge() : firstEdge());
    }

    Edge* rightEdge() const
    {
        return !isVerticalZigZag() ? secondEdge() : (firstEdge()->firstPoint() == firstPoint() ? firstEdge() : secondEdge());
    }

    // Traverses the loop in the order it was given
    Edge* nextEdge() const { return next_; }
    Edge* previousEdge() const { return previous_; }

    // Traverses the loop a specific direction
    Edge* clockwiseEdge() const { return clockwise_; }
    Edge* counterClockwiseEdge() const { return counterclockwise_; }

    bool isOuterLoop() const { return isOuterLoop_; }

    bool isVertical() const { return isVertical_; }

    // Finds vertical edges that are not
    // an opening or closing edge of a loop
    //
    // These are never the opening or closing
    // sides of loops; they're always on the
    // top or bottom side of a loop.
    bool isVerticalZigZag() const
    {
        if (!isVertical())
            return false;

        /* Check for one of these two cases
         *
         *    *->-  ->-*
         *    |        |
         *    ^        ^
         *    |        |
         * ->-*        *->-
         *
         */
        return (secondEdge()->secondPoint() == secondPoint() && firstEdge()->firstPoint() == firstPoint()) ||
               (secondEdge()->firstPoint() == secondPoint() && firstEdge()->secondPoint() == firstPoint());
    }

    // Always finds the 'bottom' side
    // of a loop start, to guarantee it's
    // found asap
    bool isLoopStart() const
    {
        return !isVertical() && !firstEdge()->isVerticalZigZag() && firstPoint() == firstEdge()->firstPoint() && angleLessThan(firstEdge());
    }

    // Not guaranteed to find any
    // particular top or bottom of an
    // loop, but always guarantees that it's
    // the last edge on the loop processed
    bool isLoopEnd() const { return nextEdge()->processed() && previousEdge()->processed(); }

    bool processed() const { return hasBeenProcessed_; }

    void setProcessed() { hasBeenProcessed_ = true; }

    geometry::Point2d unit() const { return polygon_decomposer::unit(geometry::Segment2d(firstPoint().point(), secondPoint().point())); }

  private:
    // Vertices that the edge goes between.
    // Will be eventually sorted so first is the bottom-leftmost point.
    // These link into the eventual result dcel from the algorithm.
    dcel::Vertex first_;
    dcel::Vertex second_;

    // Pointers to the adjacent edges on whatever loop this edge
    // is a part of
    Edge *previous_ = nullptr, *next_ = nullptr;

    // Pointers to the adjacent edges in a specific direction on
    // whatever loop this edge is on
    Edge *clockwise_ = nullptr, *counterclockwise_ = nullptr;

    // Set to true if the edge was created
    // from the outer loop
    bool isOuterLoop_ = false;

    // Cached value to avoid having to compute it
    // a lot
    bool isVertical_ = false;

    // Tracks whether or not we've handled this edge yet in the
    // main algorithm loop
    bool hasBeenProcessed_ = false;

    bool angleLessThan(const Edge* other) const { return angleLessThan(*other); }
    bool angleLessThan(const Edge& other) const { return unitDotWithHorizontal() < other.unitDotWithHorizontal(); }

    // TODO: Not guaranteed to sort correctly when
    // two edges on separate loops have the same first
    // point and same angle
    bool operator<(const Edge* other) const { return operator<(*other); }
    bool operator<(const Edge& other) const
    {
        if (firstPoint() == other.firstPoint())
        {
            // Always process vertical zig-zagedges before
            // edge connected to their bottom
            // point; this makes sure that we're traversing
            // around the loops in order and not skipping over
            // a random vertical edge that's not a start or end of loop
            if (isVerticalZigZag())
                return true;

            if (other.isVerticalZigZag())
                return false;

            return angleLessThan(other);
        }
        return VertexLessThan()(firstPoint(), other.firstPoint());
    }

    double unitDotWithHorizontal() const { return bg::dot_product(unit(), geometry::Point2d{0, 1}); }

    friend std::vector<std::unique_ptr<Edge>> buildSortedEdgeList(const geometry::Polygon2d& poly, Dcel& dcel);
};

// List of unfinished edges sorted in sweep line order
// by the second point on the segments
//
// TODO: Maybe build on priority_queue
class UnfinishedEdgeList
{
    std::vector<Edge*> m_edges;

  public:
    size_t size() const { return m_edges.size(); }
    void insert(Edge* e)
    {
        const auto insertIt = std::lower_bound(m_edges.begin(), m_edges.end(), e, [](const Edge* e1, const Edge* e2) {
            return VertexLessThan()(e1->secondPoint(), e2->secondPoint());
        });
        m_edges.insert(insertIt, e);
    }

    Edge* next() { return m_edges.front(); }
    void pop() { m_edges.erase(m_edges.begin()); }
};

class ActiveEdgeList
{
    std::vector<Edge*> m_edges;

    // Finds any intersection between two lines
    // The only time there are more than 1 is if the lines are colinear
    static geometry::Point2d anyIntersection(const geometry::ConstReferringSegment2d& s1, const geometry::Segment2d& s2)
    {
        bg::model::multi_point<geometry::Point2d> intersections;
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
        bool operator()(const Edge* l, const Edge* r)
        {
            const auto l1 = l->firstPoint().point();
            const auto l2 = l->secondPoint().point();

            const auto r1 = r->firstPoint().point();
            const auto r2 = r->secondPoint().point();

            // First check if one is just wholly above the other
            double minYL = l1.y();
            double maxYL = l2.y();
            if (minYL > maxYL)
                std::swap(minYL, maxYL);

            double minYR = r1.y();
            double maxYR = r2.y();
            if (minYR > maxYR)
                std::swap(minYR, maxYR);

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

            auto minXL = l1.x();
            auto maxXL = l2.x();
            if (maxXL < minXL)
                std::swap(minXL, maxXL);

            auto minXR = r1.x();
            auto maxXR = r2.x();
            if (maxXR < minXR)
                std::swap(minXR, maxXR);

            const auto sharedRangeStart = std::max(minXL, minXR);
            const auto sharedRangeEnd   = std::min(maxXL, maxXR);

            const auto minY = std::min(minYR, minYL) - 1;
            const auto maxY = std::max(maxYR, maxYL) + 1;

            ASSERT(sharedRangeStart <= sharedRangeEnd);

            geometry::Segment2d vertical{{sharedRangeStart, minY}, {sharedRangeStart, maxY}};

            const auto p1L = anyIntersection({l1, l2}, vertical);
            const auto p1R = anyIntersection({r1, r2}, vertical);

            // If the first points are the same, check the second; otherwise just check if one
            // is under the other
            if (std::abs(p1L.y() - p1R.y()) < EPSILON)
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

  public:
    long size() const { return static_cast<long>(m_edges.size()); }
    long insert(Edge* e)
    {
        const auto currEdgeInsertIt = std::lower_bound(m_edges.begin(), m_edges.end(), e, ActiveEdgeLessThan());
        const auto activeEdgeIndex  = std::distance(m_edges.begin(), currEdgeInsertIt);

        // Prevent double insertions
        if (currEdgeInsertIt == m_edges.end() || *currEdgeInsertIt != e)
            m_edges.insert(currEdgeInsertIt, e);

        return activeEdgeIndex;
    }

    void removeLessThanEqual(const Edge* e)
    {
        // Remove edges that are strictly to the left
        removeLessThan(e);

        // Remove the edge attached to this one
        // that is 'left' of it
        removeEdgeToLeft(e);
    }

    void removeEdgeToLeft(const Edge* e)
    {
        // If this edge isn't the top edge of
        // the start of a loop, we can just remove the edge
        // connected to the first point

        // Special Case:
        if (e->isVerticalZigZag() && e->secondEdge()->secondPoint() == e->secondPoint())
            removeSpecific(e->secondEdge());

        if (!(e->firstEdge()->isLoopStart() && e->firstEdge()->firstPoint() == e->firstPoint()))
            removeSpecific(e->firstEdge());
    }

    void removeLessThan(const Edge* e) { removeLessThan(e->firstPoint().point()); }

    void removeLessThan(const geometry::Point2d& p)
    {
        m_edges.erase(std::remove_if(m_edges.begin(), m_edges.end(), [&](const Edge* e) { return (e->secondPoint().point().x() < p.x()); }),
                      m_edges.end());
    }

    void removeSpecific(const Edge* e)
    {
        const auto targetIt = std::find(m_edges.begin(), m_edges.end(), e);
        if (targetIt != m_edges.end())
            m_edges.erase(targetIt);
    }

    // Returns the index of the given edge if it is present, otherwise
    // -1
    long indexOf(const Edge* e)
    {
        const auto edgeIt = std::lower_bound(m_edges.begin(), m_edges.end(), e, ActiveEdgeLessThan());
        if (edgeIt != m_edges.end() && *edgeIt == e)
            return std::distance(m_edges.begin(), edgeIt);
        return -1;
    }

    // Returns the index of the edge which is (or would be) below
    // the given edge. If the given edge is or would be the first edge
    // in the list, -1 is returned
    long indexBelow(const Edge* e)
    {
        const auto edgeIt = std::lower_bound(m_edges.begin(), m_edges.end(), e, ActiveEdgeLessThan());
        return std::distance(m_edges.begin(), std::prev(edgeIt));
    }

    Edge* operator[](const long index) { return m_edges[static_cast<size_t>(index)]; }
};

class VerticalEdgeList
{
    std::vector<dcel::HalfEdge> m_edges;

    struct VerticalEdgeLessThan
    {
        bool operator()(const dcel::HalfEdge e, const dcel::Vertex v) { return e.origin().point().x() < v.point().x(); }
        bool operator()(const dcel::Vertex v, const dcel::HalfEdge e) { return v.point().x() < e.origin().point().x(); }
        bool operator()(const dcel::HalfEdge l, const dcel::HalfEdge r) { return operator()(l, r.origin()); }
    };

  public:
    // Should always insert the downward edge
    void insert(dcel::HalfEdge edge)
    {
        ASSERT(edge && edge.twin());
        ASSERT(std::abs(edge.origin().point().x() - edge.twin().origin().point().x()) < EPSILON);
        ASSERT(edge.origin().point().y() >= edge.twin().origin().point().y());

        const auto insertIt = std::lower_bound(m_edges.begin(), m_edges.end(), edge, VerticalEdgeLessThan());
        m_edges.insert(insertIt, edge);
    }

    dcel::HalfEdge getEdge(const dcel::Vertex v1, const dcel::Vertex v2)
    {
        auto firstIt      = std::lower_bound(m_edges.begin(), m_edges.end(), v1, VerticalEdgeLessThan());
        const auto lastIt = std::upper_bound(m_edges.begin(), m_edges.end(), v1, VerticalEdgeLessThan());

        for (; firstIt != lastIt; firstIt++)
        {
            if ((firstIt->origin() == v1 && firstIt->twin().origin() == v2) || (firstIt->origin() == v2 && firstIt->twin().origin() == v1))
                return *firstIt;
        }

        return dcel::HalfEdge();
    }
};

std::vector<std::unique_ptr<Edge>> buildSortedEdgeList(const geometry::Polygon2d& poly, Dcel& dcel)
{
    typedef std::unique_ptr<Edge> EdgePtr;
    std::vector<EdgePtr> edges;

    //  a. Build list of edges such that
    //      * Every segment's first and second points are sorted
    //      * Every segment's verticies point into the final result dcel
    //      * Every segment points to the one before and after it when traversing the polygon

    // Put exterior ring first so that it'll be ring index 0
    std::vector<bg::ring_type<geometry::Polygon2d>::type> rings{poly.outer()};
    rings.insert(rings.end(), poly.inners().begin(), poly.inners().end());

    for (size_t i = 0; i < rings.size(); i++)
    {
        const auto& ring = rings[i];

        if (ring.size() < 3)
            throw std::invalid_argument("empty loop");

        // Track how many edges were in the list before we started this loop
        // so we can iterate through that group at the end to link them up
        const auto firstEdge = edges.size();

        bg::for_each_segment(ring, [&](const geometry::ConstReferringSegment2d& segment) {
            // Create segment with only first point because next segment will
            // have the second point and it will link up at the end
            //
            // Only care about the first point in the segment, because the next segment
            // with have this segment's second point as its first
            const auto& newEdge   = *edges.emplace(edges.end(), new Edge);
            newEdge->first_       = dcel.vertex(segment.first);
            newEdge->second_      = dcel.vertex(segment.second);
            newEdge->isOuterLoop_ = i == 0;

            // Sort the points for the segment based on sweep-line order
            if (!VertexLessThan()(newEdge->first_, newEdge->second_))
                std::swap(newEdge->first_, newEdge->second_);

            newEdge->isVertical_ = VertexXCoordEqual()(newEdge->first_, newEdge->second_);
        });

        // Link everything up
        for (auto j = edges.size() - 1, i = firstEdge; i < edges.size(); j = i++)
        {
            auto& edge1 = edges[j];
            auto& edge2 = edges[i];

            edge1->next_     = edge2.get();
            edge2->previous_ = edge1.get();

            if (edge1->isOuterLoop_)
            {
                edge1->clockwise_        = edge2.get();
                edge2->counterclockwise_ = edge1.get();
            }
            else
            {
                edge1->counterclockwise_ = edge2.get();
                edge2->clockwise_        = edge1.get();
            }
        }
    }

    //  b. Sort segments
    //      Sort by leftmost point, unless they're the same, then sort by other point. Points being the 'same'
    //      can be checked by seeing if they're the same vertex pointer, becuase we merged points that are physically
    //      close to each other in the last step
    std::sort(edges.begin(), edges.end(), [](const EdgePtr& l, const EdgePtr& r) { return *l < *r; });

    return edges;
}

dcel::Vertex intersection(const geometry::Segment2d e1, const geometry::Segment2d e2, Dcel& dcel)
{
    bg::model::multi_point<geometry::Point2d> intersectionPoints;

    bg::intersection(e1, e2, intersectionPoints);

    if (intersectionPoints.size() != 1)
        throw std::invalid_argument("invalid polygon: #intersections == " + std::to_string(intersectionPoints.size()));

    const auto intersectionVertex = dcel.vertex(intersectionPoints[0]);

    return intersectionVertex;
}

dcel::Vertex intersectionAbove(const dcel::Vertex v, const dcel::HalfEdge e, Dcel& dcel)
{
    // If the edge is vertical, just return the lower point from the edge
    if (VertexXCoordEqual()(e.origin(), e.next().origin()) && VertexXCoordEqual()(v, e.origin()))
        return e.origin().point().y() < e.next().origin().point().y() ? e.origin() : e.next().origin();

    // Check the endpoints of the edge below
    // In theory, this isn't necessary and boost should
    // detect this, but there are cases where it won't
    if (VertexXCoordEqual()(v, e.origin()))
        return e.origin();

    if (VertexXCoordEqual()(v, e.next().origin()))
        return e.next().origin();

    const geometry::Point2d pointAbove(v.point().x(), std::max(e.origin().point().y(), e.next().origin().point().y()) + 1);
    const geometry::Segment2d upwardSegment(v.point(), pointAbove);
    const geometry::Segment2d segmentAbove(e.origin().point(), e.next().origin().point());

    return intersection(upwardSegment, segmentAbove, dcel);
}

dcel::Vertex intersectionBelow(const dcel::Vertex v, const dcel::HalfEdge e, Dcel& dcel)
{
    // If the edge is vertical, just return the upper point from the edge
    if (VertexXCoordEqual()(e.origin(), e.next().origin()) && VertexXCoordEqual()(v, e.origin()))
        return e.origin().point().y() > e.next().origin().point().y() ? e.origin() : e.next().origin();

    // Check the endpoints of the edge below
    // In theory, this isn't necessary and boost should
    // detect this, but there are cases where it won't
    if (VertexXCoordEqual()(v, e.origin()))
        return e.origin();

    if (VertexXCoordEqual()(v, e.next().origin()))
        return e.next().origin();

    const geometry::Point2d pointBelow(v.point().x(), std::min(e.origin().point().y(), e.next().origin().point().y()) - 1);
    const geometry::Segment2d downwardSegment(v.point(), pointBelow);
    const geometry::Segment2d segmentBelow(e.origin().point(), e.next().origin().point());

    return intersection(downwardSegment, segmentBelow, dcel);
}

Edge* adjustEdgeForIntersectionBelow(dcel::Vertex pointAbove, Edge* edgeBelow)
{

    // If there's a zig-zag below and the vertical edge is in-line
    // with the line we are drawing downwards, then back up a couple steps
    // to make sure we intersect with the top of that zig-zag
    if (VertexXCoordEqual()(pointAbove, edgeBelow->firstPoint()) && edgeBelow->firstEdge()->isVerticalZigZag() &&
        edgeBelow->firstPoint() == edgeBelow->firstEdge()->firstPoint())
    {
        edgeBelow = edgeBelow->firstEdge()->secondEdge();
    }

    // If the edge below is vertical, then it has to be on the opening
    // side of a loop; so we'll step along the top of that loop
    // in order to intersect with the edge attached to the top of that
    // vertical line
    if (edgeBelow->isVertical())
    {
        ASSERT(!edgeBelow->isVerticalZigZag());
        ASSERT(edgeBelow->firstPoint() == edgeBelow->firstEdge()->firstPoint());

        edgeBelow = edgeBelow->secondEdge();
    }

    // Should never have to worry about a zig-zag case where the vertical
    // edge is attached to the second point of edge above. If that were going
    // to be the case, updating the active edge list would have removed the edge
    // above and moved to the right to the other side of the vertical edge

    return edgeBelow;
}

Edge* adjustEdgeForIntersectionAbove(dcel::Vertex pointBelow, Edge* edgeAbove)
{
    // If there's a zig-zag above and the vertical edge is in-line
    // with the line we are drawing upwards, then back up a couple steps
    // to make sure we intersect with the bottom of that zig-zag
    if (VertexXCoordEqual()(pointBelow, edgeAbove->firstPoint()) && edgeAbove->firstEdge()->isVerticalZigZag() &&
        edgeAbove->firstPoint() == edgeAbove->firstEdge()->secondPoint())
    {
        edgeAbove = edgeAbove->firstEdge()->firstEdge();
    }

    // If the edge above is vertical, then it has to be on the closing
    // side of a loop; so we'll step along the bottom of that loop
    // in order to intersect with the edge attached to the bottom of that
    // vertical line
    if (edgeAbove->isVertical())
    {
        ASSERT(!edgeAbove->isVerticalZigZag());
        ASSERT(edgeAbove->secondPoint() == edgeAbove->secondEdge()->secondPoint());

        edgeAbove = edgeAbove->firstEdge();
    }

    // Should never have to worry about a zig-zag case where the vertical
    // edge is attached to the second point of edge above. If that were going
    // to be the case, updating the active edge list would have removed the edge
    // above and moved to the right to the other side of the vertical edge

    return edgeAbove;
}

std::pair<dcel::HalfEdge, dcel::HalfEdge> getEdgesBeforeAndAfterIntersection(Edge* edge, dcel::Vertex intersectionVertex, Dcel& dcel)
{
    // Assume that the intersection happens at the origin of the segment
    // after the segment below
    auto edgeAfterIntersection  = edge->halfEdge.next();
    auto edgeBeforeIntersection = edge->halfEdge;

    edgeAfterIntersection  = dcel.splitEdge(edgeBeforeIntersection, intersectionVertex);
    edgeBeforeIntersection = edgeAfterIntersection.previous();

    return {edgeBeforeIntersection, edgeAfterIntersection};
}

dcel::HalfEdge startLowerRegion(Edge* edgeAbove, Edge* edgeBelow, VerticalEdgeList& verticalEdges, Dcel& dcel)
{
    // The edge below should have a half edge which goes
    // clockwise on the end result region, so we will split it apart
    // if the intersection is not where it ends
    // and point it to the twin half edge coming back up
    ASSERT(edgeBelow->halfEdge);

    // The edge above should not have a half edge yet, because this
    // should be the first time it is being processed
    ASSERT(!edgeAbove->halfEdge);

    // Assumption, there are no edges to the right of edgeBelow added to the
    // dcel, so going backwards one step will move to the half-edge that connects
    // the top of the region to the bottom; the 'right' side of the region
    const auto rightRegionEdge = edgeBelow->halfEdge.previous();

    // The edge will go to/from this point, which should
    // be the tip of a hole in the overall shape
    const auto pointAbove = edgeAbove->firstPoint();

    edgeBelow = adjustEdgeForIntersectionBelow(pointAbove, edgeBelow);

    // Get the vertex object for where the vertical edge to be created
    // intersects with the edge below. If the point is not in the dcel yet,
    // it will be added.
    const auto intersectionVertex = intersectionBelow(pointAbove, edgeBelow->halfEdge, dcel);

    // Check if the edge exists already; this could happen if there are multiple holes
    // in a shape which all line up where they start
    const auto existingEdge = verticalEdges.getEdge(pointAbove, intersectionVertex);
    if (existingEdge)
        return existingEdge.twin();

    dcel::HalfEdge edgeBeforeIntersection, edgeAfterIntersection;
    std::tie(edgeBeforeIntersection, edgeAfterIntersection) = getEdgesBeforeAndAfterIntersection(edgeBelow, intersectionVertex, dcel);

    // Split the region by drawing an edge straight down from the opening point of the
    // hole. If there's already an edge there, then this will just add a twin to it.
    dcel.splitEdge(rightRegionEdge, pointAbove);
    const auto downwardHalfEdge = dcel.splitRegion(rightRegionEdge, edgeAfterIntersection);

    verticalEdges.insert(downwardHalfEdge);

    return downwardHalfEdge.twin();
}

dcel::HalfEdge endLowerRegion(Edge* edgeAbove, Edge* edgeBelow, VerticalEdgeList& verticalEdges, Dcel& dcel)
{
    // Both edges should have been processed previously, so both
    // should already have a half edge assigned
    ASSERT(edgeAbove->halfEdge);
    ASSERT(edgeBelow->halfEdge);
    ASSERT(!edgeAbove->isVertical());

    // The edge will go to/from this point, which should
    // be the tip of a hole in the overall shape
    const auto pointAbove = edgeAbove->secondPoint();

    edgeBelow = adjustEdgeForIntersectionBelow(pointAbove, edgeBelow);

    // Get the vertex object for where the vertical edge to be created
    // intersects with the edge below. If the point is not in the dcel yet,
    // it will be added.
    const auto intersectionVertex = intersectionBelow(pointAbove, edgeBelow->halfEdge, dcel);

    // Check if the edge exists already; this could happen if there are multiple holes
    // in a shape which all line up where they start
    const auto existingEdge = verticalEdges.getEdge(pointAbove, intersectionVertex);
    if (existingEdge)
        return existingEdge.twin();

    dcel::HalfEdge edgeBeforeIntersection, edgeAfterIntersection;
    std::tie(edgeBeforeIntersection, edgeAfterIntersection) = getEdgesBeforeAndAfterIntersection(edgeBelow, intersectionVertex, dcel);

    // Split the region by drawing an edge straight down from the opening point of the
    // hole. If there's already an edge there, then this will just add a twin to it.
    const auto downwardHalfEdge = dcel.splitRegion(edgeAbove->halfEdge, edgeAfterIntersection);

    verticalEdges.insert(downwardHalfEdge);

    return downwardHalfEdge.twin();
}

dcel::HalfEdge startUpperRegion(Edge* edgeAbove, dcel::HalfEdge verticalDownwardEdgeBelow, VerticalEdgeList& verticalEdges, Dcel& dcel)
{
    // The edge above should have a half edge which goes
    // clockwise on its region, so we will split it apart
    // if the intersection is not where it ends
    // and point it to the twin half edge coming back down
    ASSERT(edgeAbove->halfEdge);

    const auto pointBelow = verticalDownwardEdgeBelow.origin();

    edgeAbove = adjustEdgeForIntersectionAbove(pointBelow, edgeAbove);

    const auto intersectionVertex = intersectionAbove(pointBelow, edgeAbove->halfEdge, dcel);
    const auto existingEdge       = verticalEdges.getEdge(intersectionVertex, pointBelow);

    // Edges in vertical edge list are downward, but we need it's twin going upward
    if (existingEdge)
    {
        // Pretty sure this should never happen
        ASSERT(false);
        return existingEdge.twin();
    }

    dcel::HalfEdge edgeBeforeIntersection, edgeAfterIntersection;
    std::tie(edgeBeforeIntersection, edgeAfterIntersection) = getEdgesBeforeAndAfterIntersection(edgeAbove, intersectionVertex, dcel);

    // Reassign the half edge that the Edge tracks so that
    // if it's used for a later intersection, the correct
    // region/edges are linked
    edgeAbove->halfEdge = edgeAfterIntersection;

    // Create an edge going from the end of the edge before the intersection
    // to the beginning of the edge we used to know where to draw upwards.
    // This will close the region, and create a new region on the other side of
    // this new edge
    const auto downwardHalfEdge = dcel.splitRegion(edgeBeforeIntersection, verticalDownwardEdgeBelow);

    verticalEdges.insert(downwardHalfEdge);

    // Return the twin, which is the one going upwards
    return downwardHalfEdge.twin();
}

dcel::HalfEdge endUpperRegion(Edge* edgeAbove, Edge* edgeBelow, dcel::HalfEdge verticalUpwardEdgeBelow, VerticalEdgeList& verticalEdges,
                              Dcel& dcel)
{
    // The edge above should have a half edge which goes
    // clockwise on its region, so we will split it apart
    // if the intersection is not where it ends
    // and point it to the twin half edge coming back down
    ASSERT(edgeAbove->halfEdge);
    ASSERT(edgeBelow->halfEdge);
    ASSERT(edgeBelow->secondPoint() == verticalUpwardEdgeBelow.next().origin());

    const auto pointBelow = verticalUpwardEdgeBelow.next().origin();

    edgeAbove = adjustEdgeForIntersectionAbove(pointBelow, edgeAbove);

    const auto intersectionVertex = intersectionAbove(pointBelow, edgeAbove->halfEdge, dcel);
    const auto existingEdge       = verticalEdges.getEdge(intersectionVertex, pointBelow);

    // Edges in vertical edge list are downward, but we need it's twin going upward
    if (existingEdge)
    {
        // Pretty sure this should never happen
        //ASSERT(false);
        return existingEdge.twin();
    }

    dcel::HalfEdge edgeBeforeIntersection, edgeAfterIntersection;
    std::tie(edgeBeforeIntersection, edgeAfterIntersection) = getEdgesBeforeAndAfterIntersection(edgeAbove, intersectionVertex, dcel);

    // Reassign the half edge that the Edge tracks so that
    // if it's used for a later intersection, the correct
    // region/edges are linked
    edgeAbove->halfEdge = edgeAfterIntersection;

    // Create an edge going from the end of the edge before the intersection
    // to the beginning of the edge we used to know where to draw upwards.
    // This will close the region, and create a new region on the other side of
    // this new edge
    const auto downwardHalfEdge = dcel.splitRegion(edgeBeforeIntersection, edgeBelow->halfEdge);
    verticalEdges.insert(downwardHalfEdge);

    // At this point, there's two regions on the righthand side of the vertical split; but there
    // shouldn't be in the end result - we want those regions to be combined
    const auto upwardHalfEdge = downwardHalfEdge.twin();

    dcel.mergeRegions(verticalUpwardEdgeBelow, upwardHalfEdge, upwardHalfEdge.previous(), verticalUpwardEdgeBelow.next().next());

    // Return the twin, which is the one going upwards
    return upwardHalfEdge;
}

/*!
 * Executes the modified trapezoidal decomposition algorithm outlined
 * in "Complete coverage path planning in an agricultural environment" (Driscoll, 2011)
 * with minor adjustments to account for implementation details.
 *
 * The algorithm, as defined in that paper is denoted by comments beginning with '//!',
 * and annotations made by the author of this implementation are denoted by '//'.
 *
 * As originally described, the algorithm left some details out, so the annotations
 * serve to answer some of undefined points found during implementation. In general,
 * the following points are not explicitly stated but help immensely when trying to
 * understand the algorithm:
 * * Segments are sorted first by the 'leftmost' point, then by the 'rightmost' point
 * * The 'active edges' list is sorted from -y to +y
 * * 'upper edge' means 'an edge which has the interior region of the shape above it',
 *      'lower edge' means 'an edge which has the interior region of the shape below it'
 *
 * Current limitations of implementation:
 * * Cannot handle a polygon which does not satisfy boost::geometry::is_valid()
 */
Dcel decompose(const geometry::Polygon2d& originalPoly)
{
    geometry::Polygon2d poly;

    // Simplify the polygon with a very small tolerance
    // to remove colinear edges
    bg::simplify(originalPoly, poly, 0.000001);

    if (!bg::is_valid(poly))
        throw std::invalid_argument("boost::geometry::is_valid() failed");

    //  This will be the final result
    Dcel dcel;

    //  0. Convert the polygon to a sorted edge list
    //     edges will point to the vertices in the dcel
    //     we are building, and also point to the next and
    //     previous edges on the shape.

    // List of edges to build and sort
    std::vector<std::unique_ptr<Edge>> edges = buildSortedEdgeList(poly, dcel);

    //! 1. Initialize active edge list to empty
    //  Important: This list is kept sorted by y coordinate only.
    //      If the first point of two segments has the same y coordinate
    //      then they should be ordered by the y coordinate of their second points
    //
    //  Additionally there should always be an even number of segments in the list before the 'current'
    //  segment during the algorithm. Whenever a loop is first encountered, we add the 'lower' and 'upper'
    //  segments attached to the first point, and after that we'll always remove and add edges such that
    //  there's always a 'top' and 'bottom' edge for that loop in this list
    //
    //  TODO: Determine if everything still works if an interior loop and exterior loop have
    //      a shared segment. Pretty sure yes because of sorting by first y then second y
    ActiveEdgeList activeEdges;

    // Tracks vertical lines that were created so that
    // we can avoid creating duplicate ones
    VerticalEdgeList verticalEdges;

    //! 2. Initialize list of edges to be completed to empty
    //  At each point, we may decide it's necessary to create
    //  a new polygon by adding vertical edges going both up
    //  and down from the point. Since points are sorted bottom
    //  to top, we may not have processed the segment above that
    //  the vertical edge will extend to, so we add the new edge
    //  to this list to be completed later.
    //
    //  Each element of this list is one of the last edges on an
    //  interior loop. Its presence in this list indicates that we
    //  need to create a vertical break at the second point on that
    //  edge at some point.
    UnfinishedEdgeList unfinishedEdges;

    //! 3. Initialize the location of the last vertical to "not seen"
    //  This tracks where the last line that split out a new shape was
    //  Used purely to make sure we don't cut a polygon smaller than the
    //  swath width
    //
    //  How does this work when there's multiple open regions that didn't start
    //  at the same place? Do we just cut a vertical through them all?
    // double lastVerticalXCoord = std::numeric_limits<double>::lowest();

    const auto completeUnfinishedEdges = [&](ccpp::geometry::Point2d sweepLinePoint) {
        // If the current edge's left-most point further right than the unfinished edge's right point, then
        // we can process the unfinished edge and make a vertical up and down from it
        while (unfinishedEdges.size() > 0 && VertexLessThan()(unfinishedEdges.next()->secondPoint(), sweepLinePoint))
        {
            auto currentEdgePtr = unfinishedEdges.next();
            auto& currentEdge   = *currentEdgePtr;

            // Move the sweep line forward to remove any edges that end before the one we're
            // finishing
            activeEdges.removeLessThan(currentEdge.secondPoint().point());
            ASSERT((activeEdges.size() % 2) == 0);

            // Edges that are the end of a loop should always
            // be in scope when they are finished
            const auto currentEdgeActiveIndex = activeEdges.indexOf(currentEdgePtr);
            ASSERT(currentEdgeActiveIndex != -1);

            // The last edge processed in a loop will always
            // be attached to the other 'end of loop' segment on the second point
            const auto adjacentEdgeActiveIndex = activeEdges.indexOf(currentEdge.secondEdge());
            ASSERT(adjacentEdgeActiveIndex != -1);

            const bool isLowerEdge = currentEdgeActiveIndex < adjacentEdgeActiveIndex;
            const auto lowerEdge   = isLowerEdge ? currentEdgePtr : currentEdge.secondEdge();
            const auto upperEdge   = isLowerEdge ? currentEdge.secondEdge() : currentEdgePtr;

            const auto lowerEdgeIndex = isLowerEdge ? currentEdgeActiveIndex : adjacentEdgeActiveIndex;
            const auto upperEdgeIndex = isLowerEdge ? adjacentEdgeActiveIndex : currentEdgeActiveIndex;

            // When we finish a loop, both of the last two edges on the
            // loop should still be active
            ASSERT(lowerEdgeIndex != -1);
            ASSERT(upperEdgeIndex != -1);

            // If the lower edge is even, then the loop closes
            // in 'outside' space - either in a hole or outside
            // the outer loop; either case, we don't need to do anything
            if ((lowerEdgeIndex % 2) == 0)
            {
                unfinishedEdges.pop();
                continue;
            }

            // When we finish a loop, there should be at least one edge
            // above and below the closing edges.
            ASSERT(upperEdgeIndex < activeEdges.size() - 1);
            ASSERT(lowerEdgeIndex > 0);

            const auto edgeBelow = activeEdges[lowerEdgeIndex - 1];
            const auto edgeAbove = activeEdges[upperEdgeIndex + 1];

            dcel::HalfEdge bottomRegionEdge;

            // If there's a vertical edge closing a loop, it's
            // guaranteed to be the closing edge. In this case, we
            // create a vertical edge going down from the bottom of it,
            // and a vertical edge going up from the top.
            if (lowerEdge->isVertical())
            {
                // Close off the lower region under the hole if it's not closed already
                bottomRegionEdge = endLowerRegion(lowerEdge->firstEdge(), edgeBelow, verticalEdges, dcel).next();

                // Add the vertical edge to the new region
                dcel.splitEdge(bottomRegionEdge, lowerEdge->secondPoint());
                lowerEdge->halfEdge = bottomRegionEdge;
            }
            else
            {
                // Close off the lower region under the hole if it's not closed already
                bottomRegionEdge = endLowerRegion(lowerEdge, edgeBelow, verticalEdges, dcel);
            }

            // Close off the upper region above the hole, and merge it into the region
            // that already exists below it from closing the bottom area
            endUpperRegion(edgeAbove, upperEdge, bottomRegionEdge, verticalEdges, dcel);

            unfinishedEdges.pop();
        }
    };

    //! 4. For each segment in the sorted input list
    int i              = 0;
    auto currentEdgeIt = edges.begin();
    auto sweepLineIt   = edges.begin();

    // As we process edges which share a leftmost x coordinate,
    // we build a vertical line which splits the shape apart.
    // Since dcel edges are only assigned to Edge* objects as
    // the Edges are processed, we can't guarantee that it will
    // always be possible to make an edge going upwards. This
    // tracks the most recent vertical piece below, so that we
    // can build the edge from bottom to top correctly as we
    // process all the Edges which share the leftmost x coord
    //dcel::HalfEdge downwardEdgeBelow;

    for (; currentEdgeIt != edges.end(); currentEdgeIt++, i++)
    {
        const auto& currentEdgePtr = *currentEdgeIt;
        auto& currentEdge          = *currentEdgePtr;
        // auto& currentPoint         = currentEdge.firstPoint()->location;

        //! a. Mark the edge as processed
        //  Assumption: If we used the segment to start a loop below, then
        //  it'll already be in this set and we can just ignore it
        if (currentEdge.processed())
            continue;

        //! b. Update the location of the sweep line to the 'left' endpoint
        // This is currentPoint.x()

        //! c. If the list of new edges to be completed is not empty and the
        //! location of the new edge list falls before the left endpoint of the
        //! current edge, find the index in the active edge list of the edge the
        //! vertical comes off of, and connect the vertical to the edges immediately
        //! above and below it in the active edge list
        //  Each edge in the new edges list was started previously but it was unknown
        //  what edges were directly above and below it. Once we have moved the sweep line
        //  immediately to the right of that point, then it is guaranteed that the segments
        //  it should connect to will be in the active edges list, so we can finish processing
        //  that incomplete edge.
        //
        //  Once we connect to the edges above and below in the active edge list, then
        //  what? Do we remove them and that forms a new polygon? Probably yes.

        completeUnfinishedEdges(currentEdge.firstPoint().point());

        //! d. Update the active edge list, adding the new edge, and removing any
        //! edges that have gone out of scope
        //
        //  Assumption: 'out of scope' means 'the rightmost point on the segment has
        //  x <= the current point's x and y <= the current point's y.
        //      Reasoning: There's a couple of other potential meanings that come to mind as
        //          'obvious', but none of them work great.
        //
        //          * Rightmost point x < current point x
        //              This works ok for most cases, but it breaks when there is a vertical
        //              line. In that case, the vertical line will be processed when we hit
        //              the bottom-most point, and then we will move to the segment starting
        //              at the top-most point. The vertical line; however, stays in the active
        //              edge list, and this breaks our constraint that the list will always have
        //              an even count
        //          * Rightmost point x <= current point x
        //              This solves the issue above with vertical lines, but it breaks other things.
        //              When we want to extend a line upwards to the line above it, there needs to be
        //              something there to extend it too, but if the line we would extend it to ends
        //              at exactly the same x coordinate as we are currently processing, it will have
        //              been removed, and there will be nothing in the list 'above' the current point.
        //
        //
        //  Insertion is done to keep the list sorted by y coordinates.

        //  Remove edges <= the sweep line
        //  In almost? every single case this will be a single edge; the one directly
        //      connected to the one being processed.
        //  I think this will always be exactly 1; maybe we can do something with
        //      a binary search here; but they're sorted by the first point's y, not
        //      the second. However, since there are no intersections, it should
        //      be relatively ordered the same. Need to see how that works out
        //      with multiple edges ending at the same point.

        // If we've moved the sweep line to the right
        // then remove any edges that ended in the area
        // that was passed over, and insert all the edges
        // that start at the new location
        if (sweepLineIt == currentEdgeIt)
        {
            // Any time we move forward, we reset the vertical edge in progress
            //downwardEdgeBelow = dcel::HalfEdge();

            activeEdges.removeLessThan(currentEdgePtr.get());

            // Add all other edges that start at the current sweep line location to the
            // active edge list; this is important so that if we need to make a vertical line
            // segment to break the shape here, it doesn't go all the way up to the outer boundary
            // because we haven't processed the line directly above the current one

            // After this loop, there should always
            // be an even number of edges in the active edges list
            while (sweepLineIt != edges.end() && VertexXCoordEqual()((*sweepLineIt)->firstPoint(), currentEdge.firstPoint()))
            {
                const auto sweepEdge = sweepLineIt->get();

                // Removes the edge attached to the left side of this edge
                // (if it's not a start of a loop edge), to maintain the
                // 'activeEdges.size() is even' invariant
                activeEdges.removeEdgeToLeft(sweepEdge);
                const auto sweepEdgeActiveIndex = activeEdges.insert(sweepEdge);

                const auto leftEdge = sweepEdge->leftEdge();

                // Adds the edge to the dcel if it's next to a region that can be extended.
                // This ensures that all active edges which are not the start of a loop always
                // have a half-edge and get added to the active regions as we move across the shape
                if (!sweepEdge->halfEdge && leftEdge->halfEdge)
                {
                    const auto adjacentEdge   = leftEdge;
                    const auto nextDcelVertex = sweepEdge->rightPoint();

                    ASSERT(adjacentEdge->halfEdge);

                    // Bottom edge, goes right to left
                    if ((sweepEdgeActiveIndex % 2) == 0)
                    {
                        sweepEdge->halfEdge = dcel.splitEdge(adjacentEdge->halfEdge.previous(), nextDcelVertex);
                    }

                    // Top edge, goes left to right
                    else
                    {
                        sweepEdge->halfEdge = adjacentEdge->halfEdge.next();
                        dcel.splitEdge(sweepEdge->halfEdge, nextDcelVertex);
                    }
                }

                sweepLineIt++;
            }
        }

        ASSERT((activeEdges.size() % 2) == 0);

        // Gets the index of the current edge in the active edges list; or, if it's
        // not there, the index of the edge connected. This only happens when
        // the edge is vertical. If the vertical edge is not a zig-zag, we treat
        // it as the 'top' edge of the loop, because the edge connected below it is
        // treated as the start of the loop
        const auto currEdgeActiveIndex = [&] {
            auto tmp = activeEdges.indexOf(currentEdgePtr.get());

            if (tmp == -1)
            {
                ASSERT(currentEdge.isVertical());

                if (currentEdge.isVerticalZigZag() && currentEdge.secondEdge()->secondPoint() == currentEdge.secondPoint())
                    tmp = activeEdges.indexOf(currentEdge.firstEdge());
                else
                    tmp = activeEdges.indexOf(currentEdge.secondEdge());
            }

            return tmp;
        }();

        ASSERT(currEdgeActiveIndex >= 0);

        //! e. If neither the previous nor the next edge along the boundary has been
        //! encountered, and the current edge is currently an "upper" edge (i.e. its
        //! position in the active edge list is divisible by 2), start an exterior
        //! boundary
        //
        //  AKA "If we hit the leftmost point on an exterior loop, start tracking it as such"
        //
        //  Assumption: 'encountered' means 'in processed list'
        //
        //  Clarification: 0 is divisible by 2 for the purposes of this step
        //      Reasoning: Logically, the first segment ever processed should hit this case
        //          And if we don't count 0 as divisible by 2, we'll hit the next case, which
        //          is supposed to be for interior loops
        //const bool startOfLoop = processedSegments.count(currentEdge.next) == 0 && processedSegments.count(currentEdge.previous) == 0;
        //const bool endOfLoop   = processedSegments.count(currentEdge.next) != 0 && processedSegments.count(currentEdge.previous) != 0;

        // Finds the 'bottom' edge of all opening loops
        const bool startOfLoop = currentEdge.isLoopStart();

        // Finds the last edge of a loop to be processed;
        // could be either a top or bottom. If the rightmost
        // edge of a region is vertical, it will be the end of the loop
        const bool endOfLoop = currentEdge.isLoopEnd();

        // Start of a loop where the region transitions
        // from ouside the shape to inside, so we create a new
        // region, but do not need to make a vertical line
        // or end other regions
        if (startOfLoop && currEdgeActiveIndex % 2 == 0)
        {
            //! i. Get the next segment in the sorted input list. This represents the second
            //! edge of the boundary.
            //
            //  Correction: Get the next segment on the boundary; there could be another
            //      one in the sorted list before it
            //      If that is indeed the case, then it means there's a tangency between
            //      the inner loop and the outer loop; but we'll handle it in a 'not incorrect'
            //      manner by producing a zero-size region which is connected to an upper and
            //      lower region after it
            Edge* nextEdgePtr = currentEdge.firstEdge();

            //! ii. Construct a half-edge for it, storing the new half edge as a duplicate
            //! of the original edge
            // What is 'it' in this context? The next segment probably. What is the original
            // edge? 'it'? Or the edge we're examining in the loop through the sorted edges?
            //
            // Assumption: This means we're supposed to create half-edges for both the current
            //      edge and the next one. What do we do for their 'twin' edges? Do we just ignore
            //      the outside of the boundary because we don't care about it?

            // Create a new region using the rightmost point of the current edge, then
            // split it at the leftmost point
            auto firstDcelEdge  = dcel.createRegion(currentEdge.secondPoint(), currentEdge.firstPoint());
            auto secondDcelEdge = firstDcelEdge.next();

            // Split the second edge again at the rightmost point on the 'top' edge
            auto thirdDcelEdge = dcel.splitEdge(secondDcelEdge, nextEdgePtr->secondPoint());

            currentEdge.halfEdge  = firstDcelEdge;
            nextEdgePtr->halfEdge = secondDcelEdge;

            // For the most part, the dcel regions are extended as we move
            // across the shape with the sweep line, but when a new region starts
            // we have to make sure all edges from it that are in the active edges list
            // get half edges assigned. This means the two edges that meet at a point, and
            // one extra above if there's a vertical edge to open the shape
            if (nextEdgePtr->isVertical())
            {
                const auto topEdgePtr = nextEdgePtr->secondEdge();
                topEdgePtr->halfEdge  = thirdDcelEdge;
                dcel.splitEdge(thirdDcelEdge, topEdgePtr->secondPoint());
            }

            //! iii. Update the active edge list
            //  Yup this is obvious what I'm supposed to do here
            //
            //  We already updated it above, why do we need to do it again here?
            //  Maybe we just move the sweep line forward one? That would make some sense;
            //  because if this is the first piece of the boundary encountered, then
            //  there's no reason to process the edge that will come immediately next
            //
            //  It more likely means to add this next edge along the boundary to the
            //  active edges list without stepping the sweep line.
            //
            //  Clarification:
            //      Mark the new edge as processed at the same time

            // Next edge is going to have the same first coordinate, so it
            // will already be in the active edge list (and if it's not, it's because
            // it's vertical, and we don't want it in there anyway)
            //const auto nextEdgeActiveIndex = activeEdges.insert(nextEdgePtr);

            //ASSERT(nextEdgeActiveIndex == currEdgeActiveIndex + 1);

            //! iv. Connect the two edges together
            //  Which two edges? The segment from the loop and the next segment on the polygon?
            //  Those are already connected. The new half-edge and the segment we created it from?
            //  That doesn't make sense unless we're re-using the sorted input as part of the DCEL output.
            //
            //  Done above

            // It's possible for this edge to be both a loop start and loop end,
            // but due to the order of edge processing, it's never realized as a
            // loop end. We check for that situation here
            if (!currentEdge.secondEdge()->isVertical() && currentEdge.secondEdge()->secondPoint() == currentEdge.secondPoint())
                unfinishedEdges.insert(currentEdgePtr.get());
        }

        //! f. Else, if neither the previous nor the next edge along the boundary has been processed yet,
        //! start an interior boundary.
        //
        //  AKA "If we hit the leftmost point on an interior loop, create a vertical line and end the previous region"

        // This is the start of a loop where the sweep line
        // transitions from inside the shape to outside the shape. The
        // outside area my be inside a hole, or it may be completely outside
        // the outer boundary.
        else if (startOfLoop)
        {
            if (currEdgeActiveIndex == 0 || currEdgeActiveIndex == activeEdges.size() - 1)
                throw std::invalid_argument("invalid polygon: interior edge not between exterior edges");

            //! i. Get the next segment in the sorted input list. This represents the second edge of the boundary
            //
            //  Adjustment: Just follow clockwise around the inner loop
            //
            //  Correction: "second edge of the interior loop"

            // The start of the loop is created by the current edge and the edge
            // which shares it's leftmost point. We create a vertical break going through
            // that leftmost point, but if there's a vertical edge there as part of the original
            // shape, we connect the new vertical pieces to its top and bottom to avoid creating
            // and invalid shape
            const auto bottomEdgePtr = currentEdgePtr.get();
            const auto nextEdgePtr   = currentEdge.firstEdge();

            //! ii. Mark the second edge as processed
            // There's no reason to hit the segments we're processing
            // here again later, because even if they would be flagged
            // as the closing edge of a loop, the vertical break coming off
            // them would be in space outside the original shape that we
            // don't care about
            nextEdgePtr->setProcessed();
            bottomEdgePtr->setProcessed();

            //! iii. Update the active edge list
            //  Ah yes, let me just do an 'update'
            //  Probably same as above, add the next segment along the boundary to the list; there
            //  should always be an even number of edges in this list
            //
            // Since we're updating the active edge list whenever the sweep line moves
            // we don't need to do it here

            //! iv. Connect the two edges together
            //
            //  Clarification: Make the half-edges for these edges
            //      and connect those.

            //! v. Close the current region and start two new regions. Construct an
            //! edge connecting the left end point of the first edge on the interior boundary
            //! to the upper exterior boundary (the edge in the active edge list immediately before
            //! the first interior edge). Construct an edge connecting the left end point of the first edge
            //! on the interior boundary to the lower exterior boundary (the edge in the active edge list
            //! immediately after the second interior edge). The two edges created in this step should form
            //! a straight line.
            //
            //  Clarification: in 4.g.ii below, we have to mark the point where the loop ends as an
            //      unfinished vertical segment and finish it later. That's not necessary here. The difference
            //      is that in this case, we're using the first point of the segment, so it's guaranteed the
            //      segments above and below it will be present in the active edges list, but when the loop
            //      is closed, we use the second point on the segment, so that cannot be guaranteed
            //
            //  Correction: The vertical line may not actually connect to the exterior boundary segments;
            //      it could end up connecting to another interior loop above or below.

            const auto edgeBelow = activeEdges[currEdgeActiveIndex - 1];
            const auto edgeAbove = activeEdges[currEdgeActiveIndex + 2];

            const auto lowerRegionEdge = startLowerRegion(bottomEdgePtr, edgeBelow, verticalEdges, dcel);
            bottomEdgePtr->halfEdge    = lowerRegionEdge.next();
            dcel.splitEdge(lowerRegionEdge.next(), bottomEdgePtr->secondPoint());

            auto downwardEdge = lowerRegionEdge.twin();

            // If this happens, then we're treating one
            // of the vertical zig-zag lines as a loop begin, and
            // we don't want to do that.

            ASSERT(edgeAbove != nextEdgePtr);

            // For the most part, the dcel regions are extended as we move
            // across the shape with the sweep line, but when a new region starts
            // we have to make sure all edges from it that are in the active edges list
            // get half edges assigned. This means the two edges that meet at a point, and
            // one extra above if there's a vertical edge to open the shape
            if (nextEdgePtr->isVertical())
            {
                downwardEdge          = dcel.splitEdge(downwardEdge.previous(), nextEdgePtr->secondPoint());
                nextEdgePtr->halfEdge = downwardEdge;

                const auto topEdgePtr = nextEdgePtr->secondEdge();
                topEdgePtr->halfEdge  = dcel.splitEdge(downwardEdge.previous(), topEdgePtr->secondPoint());
            }
            else
            {
                nextEdgePtr->halfEdge = dcel.splitEdge(downwardEdge.previous(), nextEdgePtr->secondPoint());
            }

            // The edge above will have a dcel edge assigned to it, unless it is also the start
            // of a loop. If it is the start of a loop, we will process it next and create a downward
            // edge to the current point, so we don't need to make an upward edge here also. If it
            // is not the start of a loop, then we need to create an upward edge here.
            if (edgeAbove->halfEdge)
            {
                startUpperRegion(edgeAbove, downwardEdge, verticalEdges, dcel);
            }

            ASSERT(!endOfLoop);
            //if (endOfLoop)
            //unfinishedEdges.insert(currentEdgePtr.get());
        }

        //! g. Else if the current edge is the last edge to be processed on an interior boundary (i.e. both the
        //! previous and next edges have already been processed and the current edge isn't the first or last edge in the
        //! active edge list)
        //
        //  AKA "Make a vertical line whenever we close an interior shape"
        else if (endOfLoop && currEdgeActiveIndex != 0 && currEdgeActiveIndex != activeEdges.size() - 1)
        {
            //! i. Connect up the edge to the boundary, creating a new edge at the sweep location if necessary. Details of
            //! this step are given in step i. below

            //! ii. Close two regions and start a new region. Construct two edges, one from the right end point to the upper
            //! exterior boundary, and one from the right end point of the current edge to the lower exterior boundary,
            //! connecting the two edges together. Leave the edges unconnected to the upper and lower exterior boundaries as it
            //! is not yet known which edges it would be connected to at the top and bottom. Add the new edges to the list of new
            //! edges to be completed
            //
            //  Correction: The vertical line may not actually connect to the exterior boundary segments;
            //      it could end up connecting to another interior loop above or below.

            unfinishedEdges.insert(currentEdgePtr.get());
        }
        //! h. Else if the current edge is the last edge to be processed on the exterior boundary
        //
        //  Assuming there is a single exterior loop with multiple interior loops, this is the last segment
        //  processed.
        else if (endOfLoop)
        {
            //! i. Build a new edge at the location of the sweep line, connecting it to the previous edge
            //! in the active edge list (the upper exterior boundary), and to the next edge in the active edge list
            //! (the lower exterior boundary).
            //
            //  Not sure why a new vertical line is needed here; not going to do that for now

            //! ii. Connect up the edge to the previous or next edge on the boundary, as appropriate, so as to close
            //! the exterior boundary

            // TODO : Figure out what's actually supposed to happen here

            // Due to the way the DCEL object always maintains closed regions, we don't need to modify
            // the structure at all when we reach the final edge in a loop
        }

        //! i. Else
        else
        {
            //! i. If the sweep line is more than the pass-width from the previous dividing line, or if an edge created at the location
            //! of the current sweep line would fall outside the field, or if the angle between the current edge and the edge to the left
            //! is close to 180 degrees, connect up the edge to the boundary, without closing the region
            //  This is the 'modified' part of the trapizoidal algorithm. Normally you would close the region at every single point
            //  enountered, but there's so many points and they're likely approximately linear, so we just keep adding them to the side
            //  of the shape
            //
            //  What does 'fall outside the field' mean, how can that happen? Shouldn't it be less than a pass-width?
            //
            //  For the purposes of the paper, 'close to 180 degrees' was +/- 10
            //
            //  By default, we extend the edges like this, so we just need to check for the negative of this state

            //! ii. Else close the current region and start a new one. Construct a new edge parallel to the sweep line, connecting
            //! the left end point of the current edge to the previous edge in the active edge list (if the current edge is a 'lower' edge)
            //! or to the next edge in the active edge list (if the current edge is an 'upper' edge)
            //
            // TODO Maybe: Cut regions based on width
        }

        currentEdge.setProcessed();
    };

    ccpp::geometry::Point2d rightPlus1 = {edges.back()->secondPoint().point().x() + 1, edges.back()->secondPoint().point().y()};

    // Complete any unfinished edges created in the last step
    completeUnfinishedEdges(rightPlus1);

    return dcel;
}

ccpp::quantity::Degrees angleDifference(const dcel::Vertex left, const dcel::Vertex mid, const dcel::Vertex right)
{
    auto p12 = mid.point();
    auto p23 = right.point();

    bg::subtract_point(p12, left.point());
    bg::subtract_point(p23, mid.point());

    const auto angle1 = atan2(p12.y(), p12.x());
    const auto angle2 = atan2(p23.y(), p23.x());

    const auto angleDiff = std::abs(angle1 - angle2);
    auto degreesDiff     = static_cast<quantity::Degrees>(units::Radian * angleDiff);

    return degreesDiff;
}

void splitRegion(const dcel::Region region, const ccpp::quantity::Degrees angleTolerance, Dcel& dcel)
{
    // TODO maybe: Make this not necessary here
    VerticalEdgeList verticalEdges;

    // To start, find the first edge along the 'top' and 'bottom'
    // sides of the region
    dcel::HalfEdge top, bottom;

    // Also track the rightmost coordinate on the region
    // to know when we stop processing it
    double maxXCoord = 0;

    // Loop around the region, and find the left, bottom-most vertex.
    // Since the loop is clockwise, we can know that the edge before that is
    // the first edge on the 'bottom'
    auto currEdge      = region.edge();
    const auto endEdge = currEdge;

    top       = currEdge;
    maxXCoord = top.origin().point().x();
    do
    {
        if (VertexLessThan()(currEdge.origin(), top.origin()))
            top = currEdge;

        maxXCoord = std::max(maxXCoord, currEdge.origin().point().x());

        currEdge = currEdge.next();
    } while (currEdge != endEdge);

    // Back off the max x coord just slightly to give epsilon compare
    maxXCoord -= 0.000001;
    bottom = top.previous();

    // If the starting edge of the region is vertical,
    // move away until the top and bottom are not on that
    // vertical side
    while (VertexXCoordEqual()(top.origin(), top.next().origin()))
    {
        top = top.next();

        // Check for degenerate case: region is just a vertical edge
        // TODO: Get rid of these, maybe?
        if (top == bottom)
            return;
    }

    // Poor man's sweep line; since all edges
    // are guaranteed to be vertical or go to the right
    // we can do this
    while (top.next().origin().point().x() < maxXCoord || bottom.origin().point().x() < maxXCoord)
    {
        // Move the top forward
        while (top.next().origin().point().x() < maxXCoord && top.next().origin().point().x() <= bottom.origin().point().x())
        {
            if (angleDifference(top.origin(), top.next().origin(), top.next().next().origin()) >= angleTolerance)
            {
                const auto intersectionVertex    = intersectionBelow(top.next().origin(), bottom, dcel);
                const auto edgeAfterIntersection = dcel.splitEdge(bottom, intersectionVertex);

                top = top.next();
                dcel.splitRegion(edgeAfterIntersection.previous(), top);
            }
            else
            {
                top = top.next();
            }
        }

        // Move the bottom forward
        while (bottom.origin().point().x() < maxXCoord && bottom.origin().point().x() <= top.next().origin().point().x())
        {
            if (angleDifference(bottom.next().origin(), bottom.origin(), bottom.previous().origin()) >= angleTolerance)
            {
                const auto intersectionVertex    = intersectionAbove(bottom.origin(), top, dcel);
                const auto edgeAfterIntersection = dcel.splitEdge(top, intersectionVertex);

                bottom = bottom.previous();
                top    = edgeAfterIntersection;

                dcel.splitRegion(bottom, edgeAfterIntersection);
            }
            else
            {
                bottom = bottom.previous();
            }
        }
    }
}

void splitRegions(Dcel& dcel, const ccpp::quantity::Degrees angleTolerance)
{
    for (const auto region : dcel.regions())
    {
        splitRegion(region, angleTolerance, dcel);
    }
}

ModifiedTrapezoidal::ModifiedTrapezoidal(const ccpp::quantity::Degrees angleTolerance) : m_angleTolerance(angleTolerance)
{
}

Dcel ModifiedTrapezoidal::decomposePolygon(const geometry::Polygon2d& poly) const
{
    auto dcel = decompose(poly);
    splitRegions(dcel, m_angleTolerance);

    return dcel;
}
}
}
}
