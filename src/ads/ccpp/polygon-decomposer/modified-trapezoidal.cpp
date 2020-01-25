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
    dcel::vertex_t *first, *second;
    Edge *previous, *next;
};

// < operator for verticies, sorts left->right, bottom->top
const auto lessThan = [](const dcel::vertex_t& left, const dcel::vertex_t& right) {
    if (left.location.x() < right.location.x())
        return true;
    else if (left.location.x() > right.location.x())
        return false;
    else
        return left.location.y() < right.location.y();
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
 * * May not work correctly if the same point is the endpoint of > 2 segments
 *      (i.e. there is a point shared by multiple loops)
 */
DoublyConnectedEdgeList decompose(const geometry::Polygon2d& poly)
{
    //  This will be the final result
    DoublyConnectedEdgeList dcel;

    //  0. Convert the polygon to a sorted edge list
    //     edges will point to the vertices in the dcel
    //     we are building, and also point to the next and
    //     previous edges on the shape.

    // List of edges to build and sort
    std::vector<std::unique_ptr<Edge>> edges;

    {
        //  a. Build list of edges such that
        //      * Every segment's first and second points are sorted
        //      * Every segment's verticies point into the final result dcel
        //      * Every segment points to the one before and after it when traversing the polygon

        // Used to look up a point to see if we already used one that's close
        // enough to consider the same
        boost::geometry::index::rtree<dcel::vertex_t*, boost::geometry::index::quadratic<100>, vertex_index> uniquePoints;

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
                // Only care about the first point in the segment, because the next segment
                // with have this segment's second point as its first
                //
                // Check if a point close enough to be the same has been inserted yet, or
                // insert a new one, in order to have a pointer to assign
                //
                // TODO: Make EPSILON not be a magic number
                constexpr double EPSILON      = 0.00001;
                const static auto cornerDelta = bg::make<geometry::Point2d>(EPSILON, EPSILON);

                const auto& p  = segment.first;
                auto minCorner = p, maxCorner = p;
                bg::subtract_point(minCorner, cornerDelta);
                bg::add_point(maxCorner, cornerDelta);

                const bg::model::box<geometry::Point2d> searchBox(minCorner, maxCorner);

                std::array<dcel::vertex_t*, 1> pointerArr = {{nullptr}};
                uniquePoints.query(bg::index::within(searchBox) && bg::index::nearest(p, 1), pointerArr.begin());

                if (!pointerArr[0])
                {
                    dcel.vertices.emplace_back(new dcel::vertex_t);
                    dcel.vertices.back()->location = p;
                    uniquePoints.insert(dcel.vertices.back().get());
                    pointerArr[0] = dcel.vertices.back().get();
                }

                // Create segment with only first point because next segment will
                // have the second point and it will link up at the end
                edges.emplace_back(new Edge);
                edges.back()->first = pointerArr[0];
            });

            // Link everything up, and sort the points
            // for each segment
            for (auto j = edges.size() - 1, i = firstEdge; i < edges.size(); j = i++)
            {
                auto& edge1 = edges[j];
                auto& edge2 = edges[i];

                edge1->next     = edge2.get();
                edge2->previous = edge1.get();

                edge1->second = edge2->first;

                if (!lessThan(*edge1->first, *edge1->second))
                    std::swap(edge1->first, edge1->second);
            }
        }

        //  b. Sort segments
        //      Sort by leftmost point, unless they're the same, then sort by other point. Points being the 'same'
        //      can be checked by seeing if they're the same vertex pointer, becuase we merged points that are physically
        //      close to each other in the last step
        std::sort(edges.begin(), edges.end(), [&](const std::unique_ptr<Edge>& left, const std::unique_ptr<Edge>& right) {
            if (left->first == right->first)
                return lessThan(*left->second, *right->second);
            return lessThan(*left->first, *right->first);
        });
    }

    // Tracks the indices of edges which have been processed
    std::unordered_set<Edge*> processedSegments;

    //! 1. Initialize active edge list to empty
    std::vector<Edge*> activeEdges;

    //! 2. Initialize list of edges to be completed to empty
    //  At each point, we may decide it's necessary to create
    //  a new polygon by adding vertical edges going both up
    //  and down from the point. Since points are sorted bottom
    //  to top, we may not have processed the segment above that
    //  the vertical edge will extend to, so we add the new edge
    //   to this list to be completed later.
    //
    //  I think these stay in order left to right?
    std::vector<geometry::Segment2d> newEdges;

    //! 3. Initialize the location of the last vertical to "not seen"
    //  This tracks where the last line that split out a new shape was
    //  Used purely to make sure we don't cut a polygon smaller than the
    //  swath width
    double lastVerticalXCoord = std::numeric_limits<double>::lowest();

    //! 4. For each segment in the sorted input list
    for (const auto& edgePtr : edges)
    {
        const auto& currentEdge = *edgePtr;

        //! a. Mark the edge as processed
        processedSegments.insert(edgePtr.get());

        //! b. Update the location of the sweep line to the 'left' endpoint
        const double sweepLineXCoord = currentEdge.first->location.x();

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
        //
        //  TODO: Figure out if all edges in this list are guaranteed to be at same x coord
        //  gut reaction says no

        for (auto it = newEdges.begin(); it != newEdges.begin() && it->first.x() < sweepLineXCoord; it++)
        {
        }

        //! d. Update the active edge list, adding the new edge, and removing any
        //! edges that have gone out of scope
        //
        //  Assumption: 'out of scope' means 'the rightmost point on the segment has
        //  an x-coord < the current point's x coord.
        //
        //  Insertion is done to keep the list sorted by y coordinate. Currently, this will
        //  not work correctly if there points in the shape that exist on multiple loops becuase
        //  more information is need to sort the segments attached to such a point correctly.
        activeEdges.erase(std::remove_if(activeEdges.begin(), activeEdges.end(),
                                         [&](const Edge* e) { return e->second->location.x() < sweepLineXCoord; }),
                          activeEdges.end());

        const auto insertIt = std::lower_bound(activeEdges.begin(), activeEdges.end(), [](const Edge* l, const Edge* r) {
            if (l->first == r->first)
                return l->second->location.y() < r->second->location.y();
            return l->first->location.y() < r->first->location.y();
        });

        const auto activeEdgeIndex = static_cast<unsigned long>(std::distance(activeEdges.begin(), insertIt));
        activeEdges.insert(insertIt, edgePtr.get());

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

        if (startOfLoop && activeEdgeIndex % 2 == 0)
        {
            //! i. Get the next segment in the sorted input list. This represents the second
            //! edge of the boundary.
            //
            //  This only works if we sort segments first by left point, then by right point.
            //  doing so will guarantee that this is a 'downward' edge, and the next will be the
            //  'upward' edge connected at the left point

            //! ii. Construct a half-edge for it, storing the new half edge as a duplicate
            //! of the original edge
            // What is 'it' in this context? The next segment probably. What is the original
            // edge? 'it'? Or the edge we're examining in the loop through the sorted edges?

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

            //! iv. Connect the two edges together
            //  Which two edges? The segment from the loop and the next segment on the polygon?
            //  Those are already connected. The new half-edge and the segment we created it from?
            //  That doesn't make sense unless we're re-using the sorted input as part of the DCEL output.
        }

        //! f. Else, if neither the previous nor the next edge along the boundary has been processed yet,
        //! start an interior boundary.
        //
        //  AKA "If we hit the leftmost point on an interior loop, start tracking it as such"
        else if (startOfLoop)
        {
            //! i. Get the next segment in the sorted input list. This represents the second edge of the boundary

            //! ii. Mark the second edge as processed
            //  processed[nextEdge] = true
            //  Why not do this in e?

            //! iii. Update the active edge list
            //  Ah yes, let me just do an 'update'
            //  Probably same as above, add the next segment along the boundary to the list; there
            //  should always be an odd number of edges in this list

            //! iv. Connect the two edges together

            //! v. Close the current region and start two new regions. Construct an
            //! edge connecting the left end point of the first edge on the interior boundary
            //! to the upper exterior boundary (the edge in the active edge list immediately before
            //! the first interior edge). Construct an edge connecting the left end point of the first edge
            //! on the interior boundary to the lower exterior boundary (the edge in the active edge list
            //! immediately after the second interior edge). The two edges created in this step should form
            //! a straight line.
            //
            //  Simply put, make a vertical line at the leftmost point of the interior shape, and
            //  close off the regions that it's going to be part of. But this talks about extending it on
            //  both sides to the exterior boundary; what if there's another interior loop above or below it?
            //  Maybe that's accounted for the the condition stated in '4.f'?
            //
            //  See 4.g.ii, which is the process for creating the vertical line at the end of this loop. That
            //  section puts the unfinished vertical pieces into the new edges list because it's not known where
            //  they will connect; maybe we need to do that here too?
            //
            //  Correction: The vertical line may not actually connect to the exterior boundary segments;
            //      it could end up connecting to another interior loop above or below.
        }

        //! g. Else if the current edge is the last edge to be processed on an interior boundary (i.e. both the
        //! previous and next edges have already been processed and the current edge isn't the first or last edge in the
        //! active edge list)
        //
        //  Basically, make a vertical line whenever we close an interior shape
        else if (endOfLoop && activeEdgeIndex != 0 && activeEdgeIndex != activeEdges.size() - 1)
        {
            //! i. Connect up the edge to the boundary, creating a new edge at the sweep location if necessary. Details of
            //! this step are given in step i. below
            // Step 4.h.i below, maybe?
            // Why is 'boundary' used to describe every single goddamn shape in this algorithm.

            //! ii. Close two regions and start a new region. Construct two edges, one from the right end point to the upper
            //! exterior boundary, and one from the right end point of the current edge to the lower exterior boundary,
            //! connecting the two edges together. Leave the edges unconnected to the upper and lower exterior boundaries as it
            //! is not yet known which edges it would be connected to at the top and bottom. Add the new edges to the list of new
            //! edges to be completed
            //
            //  Well now I know what the new edges list is for. Why do we use that here, but not at the start of the interior loop??
            //
            //  Correction: The vertical line may not actually connect to the exterior boundary segments;
            //      it could end up connecting to another interior loop above or below.
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

            //! ii. Connect up the edge to the previous or next edge on the boundary, as appropriate, so as to close
            //! the exterior boundary
        }

        //! i. Else
        //  Gotta love numbering schemes
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

            //! ii. Else close the current region and start a new one. Construct a new edge parallel to the sweep line, connecting
            //! the left end point of the current edge to the previous edge in the active edge list (if the current edge is a 'lower' edge)
            //! or to the next edge in the active edge list (if the current edge is an 'upper' edge)
        }
    };

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
