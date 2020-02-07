#include "modified-trapezoidal.h"

#include "ads/ccpp/coordinate-transform.hpp"

#include <boost/geometry/strategies/transform.hpp>

#include <algorithm>
#include <vector>
#include <unordered_set>

namespace bg = boost::geometry;

namespace ads
{
namespace ccpp
{
namespace polygon_decomposer
{

struct vertex_index
{
    typedef geometry::Point2d result_type;
    const result_type& operator()(const dcel::vertex_t* v) const { return v->location; }
};

struct Edge
{
    // Vertices that the edge goes between.
    // Will be eventually sorted so first is the bottom-leftmost point.
    // These link into the eventual result dcel from the algorithm.
    dcel::vertex_t *first = nullptr, *second = nullptr;

    // Pointers to the adjacent edges on whatever loop this edge
    // is a part of
    Edge *previous = nullptr, *next = nullptr;

    // Pointer to the most recent dcel edge created
    // which ends at the right-most point of this segment.
    // The dcel edge for the edge attached to this one
    // should point to this half edge.
    dcel::half_edge_t* halfEdge = nullptr;
};

// < operator for verticies, sorts left->right, bottom->top
bool vertexLessThan(const dcel::vertex_t& left, const dcel::vertex_t& right)
{
    if (left.location.x() < right.location.x())
        return true;
    else if (left.location.x() > right.location.x())
        return false;
    else
        return left.location.y() < right.location.y();
};

bool edgeLessThan(const std::unique_ptr<Edge>& left, const std::unique_ptr<Edge>& right)
{
    if (left->first == right->first)
        // TODO : Sort by angle if first points match
        return vertexLessThan(*left->second, *right->second);
    return vertexLessThan(*left->first, *right->first);
}

bool activeEdgeLessThan(const Edge* l, const Edge* r)
{
    if (l->first == r->first)
        // TODO : Sort by angle if first points are equal
        return l->second->location.y() < r->second->location.y();
    return l->first->location.y() < r->first->location.y();
}

class ActiveEdgeList
{
    std::vector<Edge*> m_edges;

  public:
    size_t size() const { return m_edges.size(); }
    size_t insert(Edge* e)
    {
        const auto currEdgeInsertIt = std::lower_bound(m_edges.begin(), m_edges.end(), e, &activeEdgeLessThan);
        const auto activeEdgeIndex  = static_cast<unsigned long>(std::distance(m_edges.begin(), currEdgeInsertIt));
        m_edges.insert(currEdgeInsertIt, e);

        return activeEdgeIndex;
    }

    void removeLessThan(const Edge* e)
    {
        m_edges.erase(std::remove_if(m_edges.begin(), m_edges.end(),
                                     [&](const Edge* e2) {
                                         return e2->second->location.x() <= e->first->location.x() &&
                                                e2->second->location.y() <= e->first->location.y();
                                     }),
                      m_edges.end());
    }

    size_t indexOf(const Edge* e)
    {
        const auto edgeIt = std::lower_bound(m_edges.begin(), m_edges.end(), e, &activeEdgeLessThan);
        if (*edgeIt == e)
            return std::distance(m_edges.begin(), edgeIt);
        return m_edges.size();
    }

    Edge* operator[](const size_t index) { return m_edges[index]; }
};

bool verticalEdgeVertexLessThan(const dcel::half_edge_t* e, const dcel::vertex_t* v)
{
    return e->origin->location.x() < v->location.x();
}

bool vertexVerticalEdgeLessThan(const dcel::vertex_t* v, const dcel::half_edge_t* e)
{
    return v->location.x() < e->origin->location.x();
}

bool verticalEdgeLessThan(const dcel::half_edge_t* l, const dcel::half_edge_t* r)
{
    return verticalEdgeVertexLessThan(l, r->origin);
}

class VerticalEdgeList
{
    std::vector<dcel::half_edge_t*> m_edges;

  public:
    // Should always insert the downward edge
    void insert(dcel::half_edge_t* edge)
    {
        assert(edge && edge->twin && edge->origin && edge->twin->origin);
        assert(std::abs(edge->origin->location.x() - edge->twin->origin->location.x()) < 0.00001);
        assert(edge->origin->location.y() >= edge->twin->origin->location.y());

        const auto insertIt = std::lower_bound(m_edges.begin(), m_edges.end(), edge, verticalEdgeLessThan);
        m_edges.insert(insertIt, edge);
    }

    dcel::half_edge_t* getEdge(const dcel::vertex_t* v1, const dcel::vertex_t* v2)
    {
        auto firstIt      = std::lower_bound(m_edges.begin(), m_edges.end(), v1, verticalEdgeVertexLessThan);
        const auto lastIt = std::upper_bound(m_edges.begin(), m_edges.end(), v1, vertexVerticalEdgeLessThan);

        for (; firstIt != lastIt; firstIt++)
        {
            if (((*firstIt)->origin == v1 && (*firstIt)->twin->origin == v2) ||
                ((*firstIt)->origin == v2 && (*firstIt)->twin->origin == v1))
                return *firstIt;
        }

        return nullptr;
    }
};

// Manages matching x,y points to the
// pointers for a DCEL
class DCELPointFactory
{
    // Used to look up a point to see if we already used one that's close
    // enough to consider the same
    boost::geometry::index::rtree<dcel::vertex_t*, boost::geometry::index::quadratic<25>, vertex_index> m_points;

    const double m_epsilon;
    const geometry::Point2d m_cornerDelta;

  public:
    DCELPointFactory(const double epsilon) : m_epsilon(epsilon), m_cornerDelta(m_epsilon / 2.0, m_epsilon / 2.0) {}

    dcel::vertex_t* addOrGetVertex(const geometry::Point2d& p, DoublyConnectedEdgeList& dcel)
    {
        // Check if a point close enough to be the same has been inserted yet, or
        // insert a new one, in order to have a pointer to assign
        auto minCorner = p, maxCorner = p;
        bg::subtract_point(minCorner, m_cornerDelta);
        bg::add_point(maxCorner, m_cornerDelta);

        const bg::model::box<geometry::Point2d> searchBox(minCorner, maxCorner);

        std::array<dcel::vertex_t*, 1> pointerArr = {{nullptr}};
        m_points.query(bg::index::within(searchBox) && bg::index::nearest(p, 1), pointerArr.begin());

        if (!pointerArr[0])
        {
            const auto& newVertex = *dcel.vertices.emplace(dcel.vertices.end(), new dcel::vertex_t);
            newVertex->location   = p;
            m_points.insert(newVertex.get());
            pointerArr[0] = newVertex.get();
        }
        return pointerArr[0];
    };
};

std::vector<std::unique_ptr<Edge>> buildSortedEdgeList(const geometry::Polygon2d& poly, DoublyConnectedEdgeList& dcel,
                                                       DCELPointFactory& dcelPoints)
{
    std::vector<std::unique_ptr<Edge>> edges;

    //  a. Build list of edges such that
    //      * Every segment's first and second points are sorted
    //      * Every segment's verticies point into the final result dcel
    //      * Every segment points to the one before and after it when traversing the polygon

    auto rings = poly.inners();
    rings.push_back(poly.outer());

    for (const auto& ring : rings)
    {
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
            const auto& newEdge = *edges.emplace(edges.end(), new Edge);
            newEdge->first      = dcelPoints.addOrGetVertex(segment.first, dcel);

            // Create a dcel edge to match this one
            const auto& newDcelEdge = *dcel.edges.emplace(dcel.edges.end(), new dcel::half_edge_t);
            newDcelEdge->origin     = newEdge->first;

            newEdge->halfEdge = newDcelEdge.get();
        });

        // Link everything up, and sort the points
        // for each segment
        for (auto j = edges.size() - 1, i = firstEdge; i < edges.size(); j = i++)
        {
            auto& edge1 = edges[j];
            auto& edge2 = edges[i];

            edge1->next           = edge2.get();
            edge1->halfEdge->next = edge2->halfEdge;

            edge2->previous       = edge1.get();
            edge2->halfEdge->prev = edge1->halfEdge;

            edge1->second = edge2->first;

            if (!vertexLessThan(*edge1->first, *edge1->second))
                std::swap(edge1->first, edge1->second);
        }
    }

    //  b. Sort segments
    //      Sort by leftmost point, unless they're the same, then sort by other point. Points being the 'same'
    //      can be checked by seeing if they're the same vertex pointer, becuase we merged points that are physically
    //      close to each other in the last step
    std::sort(edges.begin(), edges.end(), &edgeLessThan);

    return edges;
}

dcel::vertex_t* intersection(const geometry::ConstReferringSegment2d e1, const geometry::ConstReferringSegment2d e2,
                             DoublyConnectedEdgeList& dcel, DCELPointFactory& dcelPoints)
{
    bg::model::multi_point<geometry::Point2d> intersectionPoints;

    bg::intersection(e1, e2, intersectionPoints);

    if (intersectionPoints.size() != 1)
        throw std::invalid_argument("invalid polygon: #intersections != 1");

    const auto intersectionVertex = dcelPoints.addOrGetVertex(intersectionPoints[0], dcel);

    return intersectionVertex;
}

dcel::vertex_t* intersectionAbove(const dcel::vertex_t* v, const Edge* e, DoublyConnectedEdgeList& dcel, DCELPointFactory& dcelPoints)
{
    const geometry::Point2d pointAbove(v->location.x(), std::numeric_limits<double>::max());
    const geometry::ConstReferringSegment2d upwardSegment(v->location, pointAbove);
    const geometry::ConstReferringSegment2d segmentAbove(e->first->location, e->second->location);

    return intersection(upwardSegment, segmentAbove, dcel, dcelPoints);
}

dcel::vertex_t* intersectionBelow(const dcel::vertex_t* v, const Edge* e, DoublyConnectedEdgeList& dcel, DCELPointFactory& dcelPoints)
{

    const geometry::Point2d pointBelow(v->location.x(), std::numeric_limits<double>::lowest());
    const geometry::ConstReferringSegment2d downwardSegment(v->location, pointBelow);
    const geometry::ConstReferringSegment2d segmentBelow(e->first->location, e->second->location);

    return intersection(downwardSegment, segmentBelow, dcel, dcelPoints);
}

// Creates a downward edge from a point to the edge below it
// If such an edge already exists, it is just returned
dcel::half_edge_t* downwardEdge(dcel::vertex_t* pointAbove, Edge* edgeBelow, VerticalEdgeList& verticalEdges, DoublyConnectedEdgeList& dcel,
                                DCELPointFactory& dcelPoints)
{
    // TODO: Maybe check if x coordinates are sufficiently close to
    // be considered the same, need to see if bg::intersection sometimes fails
    // when this is the case. Ideally not

    // The edge below should have a half edge which goes
    // clockwise on the end result region, so we will split it apart
    // if the intersection is not where it ends
    // and point it to the twin half edge coming back up
    assert(edgeBelow->halfEdge != nullptr);
    assert(edgeBelow->halfEdge->next != nullptr);

    auto intersectionVertex = intersectionBelow(pointAbove, edgeBelow, dcel, dcelPoints);
    auto existingEdge       = verticalEdges.getEdge(pointAbove, intersectionVertex);
    if (existingEdge)
        return existingEdge;

    // Create the twin edges for that vertical line
    const auto& downwardHalfEdge = *dcel.edges.emplace(dcel.edges.end(), new dcel::half_edge_t);
    const auto& upwardHalfEdge   = *dcel.edges.emplace(dcel.edges.end(), new dcel::half_edge_t);

    downwardHalfEdge->twin = upwardHalfEdge.get();
    upwardHalfEdge->twin   = downwardHalfEdge.get();

    downwardHalfEdge->origin = pointAbove;
    upwardHalfEdge->origin   = intersectionVertex;

    // Assume that the intersection happens right on the edge
    // of the segment below
    auto leftOfIntersection  = edgeBelow->halfEdge->next;
    auto rightOfIntersection = edgeBelow->halfEdge;

    // If the intersection is not the origin of the half edge following
    // the one we intersected with, then it must split that edge in half;
    // so create a new half-edge representing the left side of that
    if (leftOfIntersection->origin != intersectionVertex)
    {
        const auto& newLeftOfIntersection = *dcel.edges.emplace(dcel.edges.end(), new dcel::half_edge_t);

        newLeftOfIntersection->next = leftOfIntersection;
        leftOfIntersection->prev    = newLeftOfIntersection.get();

        newLeftOfIntersection->origin = intersectionVertex;
        newLeftOfIntersection->region = leftOfIntersection->region;

        leftOfIntersection = newLeftOfIntersection.get();
    }

    rightOfIntersection->next = upwardHalfEdge.get();
    upwardHalfEdge->prev      = rightOfIntersection;

    leftOfIntersection->prev = downwardHalfEdge.get();
    downwardHalfEdge->next   = leftOfIntersection;

    downwardHalfEdge->region = leftOfIntersection->region;
    upwardHalfEdge->region   = rightOfIntersection->region;

    verticalEdges.insert(downwardHalfEdge.get());

    return downwardHalfEdge.get();
};

// Creates a downward edge from a point to the edge below it
// If such an edge already exists, it is just returned
dcel::half_edge_t* upwardEdge(dcel::vertex_t* pointBelow, Edge* edgeAbove, VerticalEdgeList& verticalEdges, DoublyConnectedEdgeList& dcel,
                              DCELPointFactory& dcelPoints)
{
    // TODO: Maybe check if x coordinates are sufficiently close to
    // be considered the same, need to see if bg::intersection sometimes fails
    // when this is the case. Ideally not

    // The edge above should have a half edge which goes
    // clockwise on its region, so we will split it apart
    // if the intersection is not where it ends
    // and point it to the twin half edge coming back down
    assert(edgeAbove->halfEdge != nullptr);
    assert(edgeAbove->halfEdge->prev != nullptr);

    auto intersectionVertex = intersectionAbove(pointBelow, edgeAbove, dcel, dcelPoints);
    auto existingEdge       = verticalEdges.getEdge(intersectionVertex, pointBelow);

    // Edges in vertical edge list are downward, but we need it's twin going upward
    if (existingEdge)
        return existingEdge->twin;

    // Create the twin edges for that vertical line
    const auto& downwardHalfEdge = *dcel.edges.emplace(dcel.edges.end(), new dcel::half_edge_t);
    const auto& upwardHalfEdge   = *dcel.edges.emplace(dcel.edges.end(), new dcel::half_edge_t);

    downwardHalfEdge->twin = upwardHalfEdge.get();
    upwardHalfEdge->twin   = downwardHalfEdge.get();

    downwardHalfEdge->origin = intersectionVertex;
    upwardHalfEdge->origin   = pointBelow;

    // Assume that the intersection happens right on the edge
    // of the segment above
    auto leftOfIntersection  = edgeAbove->halfEdge;
    auto rightOfIntersection = edgeAbove->next->halfEdge;

    // If the intersection is not the origin of the half edge following
    // the one we intersected with, then it must split that edge in half;
    // so create a new half-edge representing the left side of that
    if (rightOfIntersection->origin != intersectionVertex)
    {
        const auto& newRightOfIntersection = *dcel.edges.emplace(dcel.edges.end(), new dcel::half_edge_t);

        newRightOfIntersection->next = rightOfIntersection;
        rightOfIntersection->prev    = newRightOfIntersection.get();

        newRightOfIntersection->origin = intersectionVertex;
        newRightOfIntersection->region = rightOfIntersection->region;

        rightOfIntersection = newRightOfIntersection.get();
    }

    leftOfIntersection->next = downwardHalfEdge.get();
    downwardHalfEdge->prev   = leftOfIntersection;

    rightOfIntersection->prev = upwardHalfEdge.get();
    upwardHalfEdge->next      = rightOfIntersection;

    downwardHalfEdge->region = leftOfIntersection->region;
    upwardHalfEdge->region   = rightOfIntersection->region;

    verticalEdges.insert(downwardHalfEdge.get());

    return upwardHalfEdge.get();
};

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
DoublyConnectedEdgeList decompose(const geometry::Polygon2d& poly)
{
    if (!bg::is_valid(poly))
        throw std::invalid_argument("boost::geometry::is_valid() failed");

    // TODO: Make EPSILON not be a magic number
    constexpr double EPSILON = 0.00001;

    //  This will be the final result
    DoublyConnectedEdgeList dcel;

    // Manager for points to make sure the same pointer is used
    // for points that are spatially the same
    DCELPointFactory dcelPoints(EPSILON);

    //  0. Convert the polygon to a sorted edge list
    //     edges will point to the vertices in the dcel
    //     we are building, and also point to the next and
    //     previous edges on the shape.

    // List of edges to build and sort
    std::vector<std::unique_ptr<Edge>> edges = buildSortedEdgeList(poly, dcel, dcelPoints);

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

    // Tracks the indices of edges which have been processed
    std::unordered_set<Edge*> processedSegments;

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
    std::vector<Edge*> unfinishedEdges;

    //! 3. Initialize the location of the last vertical to "not seen"
    //  This tracks where the last line that split out a new shape was
    //  Used purely to make sure we don't cut a polygon smaller than the
    //  swath width
    //
    //  How does this work when there's multiple open regions that didn't start
    //  at the same place? Do we just cut a vertical through them all?
    double lastVerticalXCoord = std::numeric_limits<double>::lowest();

    const auto completeUnfinishedEdges = [&](const double sweepLineXCoord) {
        for (auto unfinishedEdge : unfinishedEdges)
        {
            // If the current edge's left-most point further right than the unfinished edge's right point, then
            // we can process the unfinished edge and make a vertical up and down from it
            if (unfinishedEdge->second->location.x() < sweepLineXCoord)
            {
                // Create new region to be the resulting one after closing the lower and upper regions
                // for the hole
                const auto& newDcelRegion = *dcel.regions.emplace(dcel.regions.end(), new dcel::region_t);

                const auto activeEdgeIndex = activeEdges.indexOf(unfinishedEdge);
                const bool isBottomEdge    = (activeEdgeIndex % 2) == 1;

                assert(activeEdgeIndex > 0 && activeEdgeIndex < activeEdges.size());

                const auto upperEdgeIndex = isBottomEdge ? activeEdgeIndex + 1 : activeEdgeIndex;
                const auto lowerEdgeIndex = isBottomEdge ? activeEdgeIndex : activeEdgeIndex - 1;

                assert(upperEdgeIndex < activeEdges.size() - 1);
                assert(lowerEdgeIndex > 0);

                const auto lowerEdge = activeEdges[lowerEdgeIndex];
                const auto upperEdge = activeEdges[upperEdgeIndex];

                const auto lowerDcelEdge = lowerEdge->halfEdge;
                const auto upperDcelEdge = upperEdge->halfEdge;

                const auto edgeBelow = activeEdges[lowerEdgeIndex - 1];
                const auto edgeAbove = activeEdges[upperEdgeIndex + 1];

                auto downward        = downwardEdge(unfinishedEdge->second, edgeBelow, verticalEdges, dcel, dcelPoints);
                downward->twin->next = lowerDcelEdge;
                lowerDcelEdge->prev  = downward->twin;

                downward->twin->region = downward->prev->region = newDcelRegion.get();
                newDcelRegion->edge                             = downward->twin;

                auto upward         = upwardEdge(unfinishedEdge->second, edgeAbove, verticalEdges, dcel, dcelPoints);
                upward->twin->prev  = upperDcelEdge;
                upperDcelEdge->next = upward->twin;

                upward->region = upward->next->region = newDcelRegion.get();

                upward->prev         = downward->twin;
                downward->twin->next = upward;
            }
        }
    };

    //! 4. For each segment in the sorted input list
    for (auto currentEdgeIt = edges.begin(); currentEdgeIt != edges.end(); currentEdgeIt++)
    {
        const auto& currentEdgePtr = *currentEdgeIt;
        auto& currentEdge          = *currentEdgePtr;
        auto& currentPoint         = currentEdge.first->location;

        //! a. Mark the edge as processed
        //  Assumption: If we used the segment to start a loop below, then
        //  it'll already be in this set and we can just ignore it
        if (!processedSegments.insert(currentEdgePtr.get()).second)
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

        completeUnfinishedEdges(currentPoint.x());

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
        activeEdges.removeLessThan(currentEdgePtr.get());

        // Add current edge to active list, tracking the index it was inserted at
        const auto currEdgeActiveIndex = activeEdges.insert(currentEdgePtr.get());

        // Update current edge to be a part of whatever region the edge to the
        // left of it. It's odd/even state in the active edges list will determine
        // if it's an edge going left to right or right to left
        if (currEdgeActiveIndex % 2 == 0)
            currentEdge.halfEdge->region = currentEdge.halfEdge->next->region;
        else
            currentEdge.halfEdge->region = currentEdge.halfEdge->prev->region;

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
        const bool startOfLoop = processedSegments.count(currentEdge.next) == 0 && processedSegments.count(currentEdge.previous) == 0;
        const bool endOfLoop   = processedSegments.count(currentEdge.next) != 0 && processedSegments.count(currentEdge.previous) != 0;

        if (startOfLoop && currEdgeActiveIndex % 2 == 0)
        {
            //! i. Get the next segment in the sorted input list. This represents the second
            //! edge of the boundary.
            //
            //  Correction: Get the next segment on the boundary; there could be another
            //      one in the sorted list before it
            Edge* nextEdgePtr = currentEdge.next;

            //! ii. Construct a half-edge for it, storing the new half edge as a duplicate
            //! of the original edge
            // What is 'it' in this context? The next segment probably. What is the original
            // edge? 'it'? Or the edge we're examining in the loop through the sorted edges?
            //
            // Assumption: This means we're supposed to create half-edges for both the current
            //      edge and the next one. What do we do for their 'twin' edges? Do we just ignore
            //      the outside of the boundary because we don't care about it?

            // Since we're opening up a new loop, we add a region to be
            // the inside of it
            const auto& currDcelRegion = *dcel.regions.emplace(dcel.regions.end(), new dcel::region_t);

            // Link up half-edge for bottom side of exterior loop to new region
            const auto& currDcelEdge = currentEdge.halfEdge;
            currDcelEdge->region     = currDcelRegion.get();

            // Assign first edge to the region
            currDcelRegion->edge = currDcelEdge;

            // Link half-edge for upper side of exterior loop
            const auto& nextDcelEdge = nextEdgePtr->halfEdge;
            nextDcelEdge->region     = currDcelRegion.get();

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
            const auto nextEdgeActiveIndex = activeEdges.insert(nextEdgePtr);

            assert(nextEdgeActiveIndex == currEdgeActiveIndex + 1);

            processedSegments.insert(nextEdgePtr);

            //! iv. Connect the two edges together
            //  Which two edges? The segment from the loop and the next segment on the polygon?
            //  Those are already connected. The new half-edge and the segment we created it from?
            //  That doesn't make sense unless we're re-using the sorted input as part of the DCEL output.
            //
            //  Done above
        }

        //! f. Else, if neither the previous nor the next edge along the boundary has been processed yet,
        //! start an interior boundary.
        //
        //  AKA "If we hit the leftmost point on an interior loop, create a vertical line and end the previous region"
        else if (startOfLoop)
        {
            if (currEdgeActiveIndex == 0 || currEdgeActiveIndex == activeEdges.size() - 1)
                throw std::invalid_argument("invalid polygon: interior edge not between exterior edges");

            //! i. Get the next segment in the sorted input list. This represents the second edge of the boundary
            //
            //  Adjustment: Just follow clockwise around the inner loop
            //
            //  Correction: "second edge of the interior loop"

            // Use previous becuase inner loops are counterclockwise
            const auto& nextEdgePtr = currentEdge.previous;

            //! ii. Mark the second edge as processed
            //
            //  Wasn't explicitly listed in part 'e' above, but I added it
            //  there because it's here and that makes sense
            processedSegments.insert(nextEdgePtr);

            //! iii. Update the active edge list
            //  Ah yes, let me just do an 'update'
            //  Probably same as above, add the next segment along the boundary to the list; there
            //  should always be an even number of edges in this list
            const auto nextEdgeActiveIndex = activeEdges.insert(nextEdgePtr);

            assert(nextEdgeActiveIndex == currEdgeActiveIndex + 1);

            //! iv. Connect the two edges together
            //
            //  Clarification: Make the half-edges for these edges
            //      and connect those.

            // Get the half edges for the starting edges of the interior loop
            const auto& currDcelEdge = currentEdge.halfEdge;
            const auto& nextDcelEdge = nextEdgePtr->halfEdge;

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

            // Create two new regions, and associate them with the
            // edges created above
            const auto& upperDcelRegion = *dcel.regions.emplace(dcel.regions.end(), new dcel::region_t);
            const auto& lowerDcelRegion = *dcel.regions.emplace(dcel.regions.end(), new dcel::region_t);

            const auto edgeBelow = activeEdges[currEdgeActiveIndex - 1];
            const auto edgeAbove = activeEdges[currEdgeActiveIndex + 1];

            auto downward        = downwardEdge(currentEdge.first, edgeBelow, verticalEdges, dcel, dcelPoints);
            downward->twin->next = currDcelEdge;
            currDcelEdge->prev   = downward->twin;

            downward->twin->region = downward->twin->next->region = downward->twin->prev->region = lowerDcelRegion.get();
            lowerDcelRegion->edge                                                                = downward->twin;

            auto upward        = upwardEdge(currentEdge.first, edgeAbove, verticalEdges, dcel, dcelPoints);
            upward->prev       = nextDcelEdge;
            nextDcelEdge->next = upward;

            upward->region = upward->next->region = upward->prev->region = upperDcelRegion.get();
            upperDcelRegion->edge                                        = upward;

            downward->prev     = upward->twin;
            upward->twin->next = downward;
        }

        //! g. Else if the current edge is the last edge to be processed on an interior boundary (i.e. both the
        //! previous and next edges have already been processed and the current edge isn't the first or last edge in the
        //! active edge list)
        //
        //  AKA "Make a vertical line whenever we close an interior shape"
        else if (endOfLoop && (currEdgeActiveIndex != 0 || currEdgeActiveIndex != activeEdges.size() - 1))
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

            // Get the next and previous edges going around this inner loop
            // clockwise (it's flipped because inner loops are counter clockwise)
            unfinishedEdges.push_back(currentEdgePtr.get());
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

            assert(currentEdgeIt + 1 == edges.end());

            // Complete any unfinished edges created in the last step
            completeUnfinishedEdges(currentEdge.second->location.x() + 1);
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
            // TODO: Cut regions based on width
        }
    };

    // Remove any unused regions in the dcel; this is rare, but can happen
    std::unordered_set<const dcel::region_t*> unusedRegions;
    for (const auto& region : dcel.regions)
        unusedRegions.insert(region.get());

    for (const auto& edge : dcel.edges)
        unusedRegions.erase(edge->region);

    dcel.regions.erase(std::remove_if(dcel.regions.begin(), dcel.regions.end(),
                                      [&](const std::unique_ptr<dcel::region_t>& region) { return unusedRegions.count(region.get()) > 0; }),
                       dcel.regions.end());

    return dcel;
}

ModifiedTrapezoidal::ModifiedTrapezoidal(const quantity::Radians sweepDir) : m_sweepDir(sweepDir)
{
}

DoublyConnectedEdgeList ModifiedTrapezoidal::decomposePolygon(const geometry::Polygon2d& poly) const
{
    // Move shape to origin, and rotate so sweep dir is positive X direction
    // TODO Maybe: Move this out of this object
    const auto transform = moveToOriginAndRotateCCWTransform(poly, -m_sweepDir);

    geometry::Polygon2d adjustedPoly;
    boost::geometry::transform(poly, adjustedPoly, transform);

    // Do decomposition
    auto dcel = decompose(poly);

    // Rotate back to original orientation?

    return dcel;
}
}
}
}
