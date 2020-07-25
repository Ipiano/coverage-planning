#include "modified-trapezoidal.h"

#include "ads/assertion.h"
#include "ads/epsilon.h"

#include "ads/ccpp/coordinate-transform.hpp"

#include "ads/algorithms/sweep-line/geometry.h"
#include "ads/algorithms/sweep-line/polygon-sweep-line.h"

#include "ads/ccpp/polygon-decomposer/modified-trapezoidal/unfinished-edge-list.h"
#include "ads/ccpp/polygon-decomposer/modified-trapezoidal/vertical-edge-list.h"
#include "ads/ccpp/polygon-decomposer/modified-trapezoidal/edge-data.h"

#include <boost/geometry/strategies/transform.hpp>

#include <algorithm>
#include <vector>
#include <unordered_set>
#include <queue>

namespace bg = boost::geometry;

using namespace ads::algorithms::sweep_line;

namespace ads
{
namespace ccpp
{
namespace polygon_decomposer
{
namespace modified_trapezoidal
{

// Declarations of a bunch of helper functions used in the algorithm; see definitions
// at bottom of file for brief descriptions of how they're used

dcel::Vertex intersection(const geometry::Segment2d e1, const geometry::Segment2d e2, Dcel& dcel);
dcel::Vertex intersectionBelow(const dcel::Vertex v, const dcel::HalfEdge e, Dcel& dcel);
dcel::Vertex intersectionAbove(const dcel::Vertex v, const dcel::HalfEdge e, Dcel& dcel);
PolygonEdge* adjustEdgeForIntersectionBelow(dcel::Vertex pointAbove, PolygonEdge* edgeBelow);
PolygonEdge* adjustEdgeForIntersectionAbove(dcel::Vertex pointBelow, PolygonEdge* edgeAbove);
std::pair<dcel::HalfEdge, dcel::HalfEdge> getEdgesBeforeAndAfterIntersection(PolygonEdge* edge, dcel::Vertex intersectionVertex,
                                                                             Dcel& dcel);
dcel::HalfEdge startLowerRegion(PolygonEdge* edgeAbove, PolygonEdge* edgeBelow, VerticalEdgeList& verticalEdges, Dcel& dcel);
dcel::HalfEdge endLowerRegion(PolygonEdge* edgeAbove, PolygonEdge* edgeBelow, VerticalEdgeList& verticalEdges, Dcel& dcel);
dcel::HalfEdge startUpperRegion(PolygonEdge* edgeAbove, dcel::HalfEdge verticalDownwardEdgeBelow, VerticalEdgeList& verticalEdges,
                                Dcel& dcel);
dcel::HalfEdge endUpperRegion(PolygonEdge* edgeAbove, PolygonEdge* edgeBelow, dcel::HalfEdge verticalUpwardEdgeBelow,
                              VerticalEdgeList& verticalEdges, Dcel& dcel);
ccpp::quantity::Degrees angleDifference(const dcel::Vertex left, const dcel::Vertex mid, const dcel::Vertex right);

/*!
 * \brief Core polygon decomposition algorithm
 *
 * This class inherits the PolygonSweepLineAlgorithm and implements the
 * virtual sweepStart(), sweepStep(), and sweepEnd() methods to perform
 * a modified trapezoidal decomposition as the sweep line moves across the
 * shape.
 *
 * Once the decomposition has been run, the result() function can be used to
 * take the result out of this object. This will move the Dcel, to avoid
 * copying a large data structure, so result() should only be called one time.
 */
class DecomposeAlgorithm : public algorithms::sweep_line::PolygonSweepLineAlgorithm
{
  public:
    //! Call exactly once after run() to take the result out of this object
    Dcel result() { return std::move(m_dcel); }

  private:
    //! The result of running the algorithm
    Dcel m_dcel;

    //! Tracker for vertical edges created to split the shape
    //! Edges are added here to ensure we don't create a duplicate
    VerticalEdgeList m_verticalEdges;

    //!  At each point, we may decide it's necessary to create
    //!  a new polygon by adding vertical edges going both up
    //!  and down from the point. Since points are sorted bottom
    //!  to top, we may not have processed the segment above that
    //!  the vertical edge will extend to, so we add the new edge
    //!  to this list to be completed later.
    //!
    //!  Each element of this list is one of the last edges on an
    //!  interior loop. Its presence in this list indicates that we
    //!  need to create a vertical break at the second point on that
    //!  edge at some point.
    UnfinishedEdgeList m_unfinishedEdges;

    //! Counter of how many edges have been processed; this is
    //! particularly useful for debugging, just to have an index into
    //! the edge list for where things are going wrong
    size_t m_edgesSeen;

    //! Tracks the iterator to the end of the edge list so we can get the
    //! correctly complete the unfinished edges
    EdgeList::const_iterator m_edgesEnd;

    void sweepBegin(EdgeList::const_iterator edgesBegin, EdgeList::const_iterator edgesEnd) override;
    void sweepStep(EdgeList::const_iterator stepBegin, EdgeList::const_iterator stepEnd,
                   const algorithms::sweep_line::ActivePolygonEdgesList& activeEdges) override;
    void sweepEnd(EdgeList::const_iterator edgesBegin, EdgeList::const_iterator edgesEnd) override;

    void completeUnfinishedEdges(ccpp::geometry::Point2d sweepLinePoint, const algorithms::sweep_line::ActivePolygonEdgesList& activeEdges);
};

//! < operator for verticies in a DCEL, sorts left->right, bottom->top
struct VertexLessThan
{
    bool operator()(const dcel::Vertex& left, const dcel::Vertex& right) { return operator()(left.point(), right.point()); };
    bool operator()(const dcel::Vertex& left, const geometry::Point2d& right) { return operator()(left.point(), right); };
    bool operator()(const geometry::Point2d& left, const dcel::Vertex& right) { return operator()(left, right.point()); }
    bool operator()(const geometry::Point2d& left, const geometry::Point2d& right)
    {
        return ads::algorithms::sweep_line::pointLessThan(left, right);
    }
};

using ads::algorithms::sweep_line::haveSameXCoord;

//! Checks if two DCEL vertices have the same X coord (within reason)
bool haveSameXCoord(const dcel::Vertex v1, const dcel::Vertex v2)
{
    return haveSameXCoord(v1.point(), v2.point());
}

/*!
 * \brief Finishes any pending vertical splits which are to the left of the sweep line point
 *
 * As the decomposition runs, vertical split points are identified at the closing sides of
 * internal loops; but they cannot be completed because the edges above and below may not yet
 * be known. This method is called right at the end of each sweep step to finish those vertical
 * splits that occur to the left of where the sweep line will be moving to on the next step.
 *
 * \param sweepLinePoint Location of the next sweep line step
 * \param originalActiveEdges List of active edges, used to know what is above or below the vertical split points
 */
void DecomposeAlgorithm::completeUnfinishedEdges(ccpp::geometry::Point2d sweepLinePoint,
                                                 const algorithms::sweep_line::ActivePolygonEdgesList& originalActiveEdges)
{
    // As we process edges in the unfinished edges list, we need to move the sweep line forward
    // to remove edges and take them out of scope a little early; but we can't modify the
    // official list, so this copy is the one used
    algorithms::sweep_line::ActivePolygonEdgesList activeEdges(originalActiveEdges);

    // If the current edge's left-most point further right than the unfinished edge's right point, then
    // we can process the unfinished edge and make a vertical up and down from it
    while (m_unfinishedEdges.size() > 0 && VertexLessThan()(m_unfinishedEdges.next()->secondPoint(), sweepLinePoint))
    {
        auto currentEdgePtr        = m_unfinishedEdges.next();
        auto& currentEdge          = *currentEdgePtr;
        const auto currentEdgeData = currentEdge.getData<EdgeData>();

        // Move the sweep line forward to remove any edges that end before the one we're
        // finishing
        activeEdges.removeLessThan(currentEdge.secondPoint());
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
            m_unfinishedEdges.pop();
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
            bottomRegionEdge = endLowerRegion(lowerEdge->firstEdge(), edgeBelow, m_verticalEdges, m_dcel).next();

            // Add the vertical edge to the new region
            const auto lowerEdgeData = lowerEdge->getData<EdgeData>();
            m_dcel.splitEdge(bottomRegionEdge, lowerEdgeData->secondVertex());
            lowerEdgeData->halfEdge() = bottomRegionEdge;
        }
        else
        {
            // Close off the lower region under the hole if it's not closed already
            bottomRegionEdge = endLowerRegion(lowerEdge, edgeBelow, m_verticalEdges, m_dcel);
        }

        // Close off the upper region above the hole, and merge it into the region
        // that already exists below it from closing the bottom area
        endUpperRegion(edgeAbove, upperEdge, bottomRegionEdge, m_verticalEdges, m_dcel);

        m_unfinishedEdges.pop();
    }
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
Dcel decompose(const geometry::Polygon2d& originalPoly)
{
    geometry::Polygon2d poly;

    // Simplify the polygon with a very small tolerance
    // to remove colinear edges
    bg::simplify(originalPoly, poly, 0.000001);

    if (!bg::is_valid(poly))
        throw std::invalid_argument("boost::geometry::is_valid() failed");

    DecomposeAlgorithm algo;
    algo.run(poly);

    return algo.result();
}

void DecomposeAlgorithm::sweepBegin(EdgeList::const_iterator edgesBegin, EdgeList::const_iterator edgesEnd)
{
    //  0. Convert the polygon to a sorted edge list
    //     edges will point to the vertices in the dcel
    //     we are building, and also point to the next and
    //     previous edges on the shape.

    //! 1. Initialize active edge list to empty
    //! 2. Initialize list of edges to be completed to empty
    //! 3. Initialize the location of the last vertical to "not seen"
    // Steps 0-3 happen before this method is called

    // Initialize the DCEL vertices for every edge in the polygon, and create
    // the 'edge data' object that tracks the decomposition-specific data for the edge
    std::for_each(edgesBegin, edgesEnd, [this](const std::unique_ptr<PolygonEdge>& edgePtr) {
        edgePtr->setData(std::make_shared<EdgeData>(edgePtr.get(), m_dcel));
    });

    m_edgesSeen = 0;
    m_edgesEnd  = edgesEnd;
}

void DecomposeAlgorithm::sweepStep(EdgeList::const_iterator stepBegin, EdgeList::const_iterator stepEnd,
                                   const algorithms::sweep_line::ActivePolygonEdgesList& activeEdges)
{
    // Move through all the edges in the step and add them into the result DCEL
    // if the edge next to them is already part of it. If it's not, then the edge
    // will be added as the first edge of a loop
    std::for_each(stepBegin, stepEnd, [=](const std::unique_ptr<PolygonEdge>& edgePtr) {
        const auto sweepEdge     = edgePtr.get();
        const auto sweepEdgeData = sweepEdge->getData<EdgeData>();

        // Need to figure out if the edge is part of the top or bottom
        // of a loop. This can be done by checking index in the active edges list.
        // However, if the edge is a vertical zig-zag, then it won't be there; so
        // we have to check its neighbor, which will have been inserted instead
        const auto sweepEdgeActiveIndex =
            sweepEdge->isVerticalZigZag() ? activeEdges.indexOf(sweepEdgeData->rightEdge()) : activeEdges.indexOf(sweepEdge);

        const auto leftEdge     = sweepEdgeData->leftEdge();
        const auto leftEdgeData = leftEdge->getData<EdgeData>();

        // Adds the edge to the dcel if it's next to a region that can be extended.
        // This ensures that all active edges which are not the start of a loop always
        // have a half-edge and get added to the active regions as we move across the shape
        if (!sweepEdgeData->halfEdge() && leftEdgeData->halfEdge())
        {
            ASSERT(sweepEdgeActiveIndex != -1);

            const auto adjacentEdge     = leftEdge;
            const auto nextDcelVertex   = sweepEdgeData->rightVertex();
            const auto adjacentEdgeData = adjacentEdge->getData<EdgeData>();

            ASSERT(adjacentEdge->getData<EdgeData>()->halfEdge());

            // Bottom edge, goes right to left
            if ((sweepEdgeActiveIndex % 2) == 0)
            {
                sweepEdgeData->halfEdge() = m_dcel.splitEdge(adjacentEdgeData->halfEdge().previous(), nextDcelVertex);
            }

            // Top edge, goes left to right
            else
            {
                sweepEdgeData->halfEdge() = adjacentEdgeData->halfEdge().next();
                m_dcel.splitEdge(sweepEdgeData->halfEdge(), nextDcelVertex);
            }
        }
    });

    //! 4. For each segment in the sorted input list...
    std::for_each(stepBegin, stepEnd, [=](const std::unique_ptr<PolygonEdge>& currentEdgePtr) {
        auto& currentEdge          = *currentEdgePtr;
        const auto currentEdgeData = currentEdge.getData<EdgeData>();

        //! a. Mark the edge as processed
        //  Assumption: If we used the segment to start a loop below, then
        //  it'll already be in this set and we can just ignore it
        if (currentEdgeData->processed())
            return;

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
        //  This is moved to the end of the previous sweep line step instead of here at the
        //  beginning of the current one. This
        //! d. Update the active edge list, adding the new edge, and removing any
        //! edges that have gone out of scope
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

                if (currentEdge.isVerticalZigZag() &&
                    currentEdge.secondEdge()->getData<EdgeData>()->secondVertex() == currentEdgeData->secondVertex())
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
        const bool startOfLoop = currentEdgeData->isLoopStart();

        // Finds the last edge of a loop to be processed;
        // could be either a top or bottom. If the rightmost
        // edge of a region is vertical, it will be the end of the loop
        const bool endOfLoop = currentEdgeData->isLoopEnd();

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
            PolygonEdge* nextEdgePtr = currentEdge.firstEdge();
            const auto nextEdgeData  = nextEdgePtr->getData<EdgeData>();

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
            auto firstDcelEdge  = m_dcel.createRegion(currentEdgeData->secondVertex(), currentEdgeData->firstVertex());
            auto secondDcelEdge = firstDcelEdge.next();

            // Split the second edge again at the rightmost point on the 'top' edge
            auto thirdDcelEdge = m_dcel.splitEdge(secondDcelEdge, nextEdgeData->secondVertex());

            currentEdgeData->halfEdge() = firstDcelEdge;
            nextEdgeData->halfEdge()    = secondDcelEdge;

            // For the most part, the dcel regions are extended as we move
            // across the shape with the sweep line, but when a new region starts
            // we have to make sure all edges from it that are in the active edges list
            // get half edges assigned. This means the two edges that meet at a point, and
            // one extra above if there's a vertical edge to open the shape
            if (nextEdgePtr->isVertical())
            {
                const auto topEdgePtr  = nextEdgePtr->secondEdge();
                const auto topEdgeData = topEdgePtr->getData<EdgeData>();

                topEdgeData->halfEdge() = thirdDcelEdge;
                m_dcel.splitEdge(thirdDcelEdge, topEdgeData->secondVertex());
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
            if (!currentEdge.secondEdge()->isVertical() &&
                currentEdge.secondEdge()->getData<EdgeData>()->secondVertex() == currentEdgeData->secondVertex())
                m_unfinishedEdges.insert(currentEdgePtr.get());
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
            const auto bottomEdgePtr  = currentEdgePtr.get();
            const auto bottomEdgeData = bottomEdgePtr->getData<EdgeData>();

            const auto nextEdgePtr  = currentEdge.firstEdge();
            const auto nextEdgeData = nextEdgePtr->getData<EdgeData>();

            //! ii. Mark the second edge as processed
            // There's no reason to hit the segments we're processing
            // here again later, because even if they would be flagged
            // as the closing edge of a loop, the vertical break coming off
            // them would be in space outside the original shape that we
            // don't care about
            nextEdgeData->setProcessed();
            bottomEdgeData->setProcessed();

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

            const auto lowerRegionEdge = startLowerRegion(bottomEdgePtr, edgeBelow, m_verticalEdges, m_dcel);
            bottomEdgeData->halfEdge() = lowerRegionEdge.next();
            m_dcel.splitEdge(lowerRegionEdge.next(), bottomEdgeData->secondVertex());

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
                downwardEdge             = m_dcel.splitEdge(downwardEdge.previous(), nextEdgeData->secondVertex());
                nextEdgeData->halfEdge() = downwardEdge;

                const auto topEdgePtr  = nextEdgePtr->secondEdge();
                const auto topEdgeData = topEdgePtr->getData<EdgeData>();

                topEdgeData->halfEdge() = m_dcel.splitEdge(downwardEdge.previous(), topEdgeData->secondVertex());
            }
            else
            {
                nextEdgeData->halfEdge() = m_dcel.splitEdge(downwardEdge.previous(), nextEdgeData->secondVertex());
            }

            // The edge above will have a dcel edge assigned to it, unless it is also the start
            // of a loop. If it is the start of a loop, we will process it next and create a downward
            // edge to the current point, so we don't need to make an upward edge here also. If it
            // is not the start of a loop, then we need to create an upward edge here.
            if (edgeAbove->getData<EdgeData>()->halfEdge())
            {
                startUpperRegion(edgeAbove, downwardEdge, m_verticalEdges, m_dcel);
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

            m_unfinishedEdges.insert(currentEdgePtr.get());
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

        currentEdgeData->setProcessed();
    });

    // Need to complete unfinished edges here, before moving on to the
    // next step, because that will remove edges from the activeEdges list
    if (stepEnd != m_edgesEnd)
    {
        // Finish all edges up to the start of the next step
        completeUnfinishedEdges((*stepEnd)->firstPoint(), activeEdges);
    }
    else
    {
        // Finish all the remaining edges
        if (m_unfinishedEdges.size())
        {
            Point2d afterEnd = m_unfinishedEdges.last()->secondPoint();
            bg::add_point(afterEnd, Point2d(1, 0));
            completeUnfinishedEdges(afterEnd, activeEdges);
        }
    }
}

void DecomposeAlgorithm::sweepEnd(EdgeList::const_iterator, EdgeList::const_iterator)
{
}

/*!
 * \brief Splits a region by adding vertical lines at inflection points
 *
 * The original QuickOPP algorithm was defined such that the angles between
 * adjacent edges were considered during the sweep line step and used to create
 * extra split points in the shape. To keep the decomposition algorithm more
 * straightforward and well-defined, that step is deferred until the end, after
 * a full decomposition is completed.
 *
 * This method takes a region of the decomposed polygon and sweeps across it again,
 * splitting the region with a vertical edge anywhere adjacent edges form too sharp of
 * an angle, as defined by the input tolerance.
 *
 * \param region Region to split apart
 * \param angleTolerance Max allowed angle difference from one edge to the next. The angle
 * between the edges must be between [180-tolerance, 180+tolerance] for no split to happen.
 * \param dcel DCEL structure being produced
 */
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
    while (haveSameXCoord(top.origin(), top.next().origin()))
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

////////////////////////////// DEFINITION OF HELPER FUNCTIONS ////////////////////////////////////////////

//! Gets the intersection of two segments, assuming there is exactly 1
//! \throws std::invalid_argument if there is not exactly 1 intersection
dcel::Vertex intersection(const geometry::Segment2d e1, const geometry::Segment2d e2, Dcel& dcel)
{
    bg::model::multi_point<geometry::Point2d> intersectionPoints;

    bg::intersection(e1, e2, intersectionPoints);

    if (intersectionPoints.size() != 1)
        throw std::invalid_argument("invalid polygon: #intersections == " + std::to_string(intersectionPoints.size()));

    const auto intersectionVertex = dcel.vertex(intersectionPoints[0]);

    return intersectionVertex;
}

//! Finds the intersection point on some edge directly above a specific vertex
//! Adds the point to the dcel if it is not already a vertex there
dcel::Vertex intersectionAbove(const dcel::Vertex v, const dcel::HalfEdge e, Dcel& dcel)
{
    // If the edge is vertical, just return the lower point from the edge
    if (haveSameXCoord(e.origin(), e.next().origin()) && haveSameXCoord(v, e.origin()))
        return e.origin().point().y() < e.next().origin().point().y() ? e.origin() : e.next().origin();

    // Check the endpoints of the edge below
    // In theory, this isn't necessary and boost should
    // detect this, but there are cases where it won't
    if (haveSameXCoord(v, e.origin()))
        return e.origin();

    if (haveSameXCoord(v, e.next().origin()))
        return e.next().origin();

    const geometry::Point2d pointAbove(v.point().x(), std::max(e.origin().point().y(), e.next().origin().point().y()) + 1);
    const geometry::Segment2d upwardSegment(v.point(), pointAbove);
    const geometry::Segment2d segmentAbove(e.origin().point(), e.next().origin().point());

    return intersection(upwardSegment, segmentAbove, dcel);
}

//! Finds the intersection point on some edge directly below a specific vertex
//! Adds the point to the dcel if it is not already a vertex there
dcel::Vertex intersectionBelow(const dcel::Vertex v, const dcel::HalfEdge e, Dcel& dcel)
{
    // If the edge is vertical, just return the upper point from the edge
    if (haveSameXCoord(e.origin(), e.next().origin()) && haveSameXCoord(v, e.origin()))
        return e.origin().point().y() > e.next().origin().point().y() ? e.origin() : e.next().origin();

    // Check the endpoints of the edge below
    // In theory, this isn't necessary and boost should
    // detect this, but there are cases where it won't
    if (haveSameXCoord(v, e.origin()))
        return e.origin();

    if (haveSameXCoord(v, e.next().origin()))
        return e.next().origin();

    const geometry::Point2d pointBelow(v.point().x(), std::min(e.origin().point().y(), e.next().origin().point().y()) - 1);
    const geometry::Segment2d downwardSegment(v.point(), pointBelow);
    const geometry::Segment2d segmentBelow(e.origin().point(), e.next().origin().point());

    return intersection(downwardSegment, segmentBelow, dcel);
}

//! Finds the edge below a point that should be used to create a vertical split in the polygon
//! This sometimes needs to adjust left or right by an edge around odd points on the boundary like vertical edges
PolygonEdge* adjustEdgeForIntersectionBelow(dcel::Vertex pointAbove, PolygonEdge* edgeBelow)
{

    // If there's a zig-zag below and the vertical edge is in-line
    // with the line we are drawing downwards, then back up a couple steps
    // to make sure we intersect with the top of that zig-zag
    if (haveSameXCoord(pointAbove.point(), edgeBelow->firstPoint()) && edgeBelow->firstEdge()->isVerticalZigZag() &&
        edgeBelow->getData<EdgeData>()->firstVertex() == edgeBelow->firstEdge()->getData<EdgeData>()->firstVertex())
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
        ASSERT(edgeBelow->getData<EdgeData>()->firstVertex() == edgeBelow->firstEdge()->getData<EdgeData>()->firstVertex());

        edgeBelow = edgeBelow->secondEdge();
    }

    // Should never have to worry about a zig-zag case where the vertical
    // edge is attached to the second point of edge above. If that were going
    // to be the case, updating the active edge list would have removed the edge
    // above and moved to the right to the other side of the vertical edge

    return edgeBelow;
}

//! Finds the edge above a point that should be used to create a vertical split in the polygon
//! This sometimes needs to adjust left or right by an edge around odd points on the boundary like vertical edges
PolygonEdge* adjustEdgeForIntersectionAbove(dcel::Vertex pointBelow, PolygonEdge* edgeAbove)
{
    // If there's a zig-zag above and the vertical edge is in-line
    // with the line we are drawing upwards, then back up a couple steps
    // to make sure we intersect with the bottom of that zig-zag
    if (haveSameXCoord(pointBelow.point(), edgeAbove->firstPoint()) && edgeAbove->firstEdge()->isVerticalZigZag() &&
        edgeAbove->getData<EdgeData>()->firstVertex() == edgeAbove->firstEdge()->getData<EdgeData>()->secondVertex())
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
        ASSERT(edgeAbove->getData<EdgeData>()->secondVertex() == edgeAbove->secondEdge()->getData<EdgeData>()->secondVertex());

        edgeAbove = edgeAbove->firstEdge();
    }

    // Should never have to worry about a zig-zag case where the vertical
    // edge is attached to the second point of edge above. If that were going
    // to be the case, updating the active edge list would have removed the edge
    // above and moved to the right to the other side of the vertical edge

    return edgeAbove;
}

//! Splits an edge at some point on it and returns the 'before' and 'after' edges respectively
//! This is used when splitting the polygon, so it is called right after intersectionAbove() or intersectionBelow()
std::pair<dcel::HalfEdge, dcel::HalfEdge> getEdgesBeforeAndAfterIntersection(PolygonEdge* edge, dcel::Vertex intersectionVertex, Dcel& dcel)
{
    // Assume that the intersection happens at the origin of the segment
    // after the segment below
    auto edgeBeforeIntersection = edge->getData<EdgeData>()->halfEdge();
    auto edgeAfterIntersection  = edgeBeforeIntersection.next();

    edgeAfterIntersection  = dcel.splitEdge(edgeBeforeIntersection, intersectionVertex);
    edgeBeforeIntersection = edgeAfterIntersection.previous();

    return {edgeBeforeIntersection, edgeAfterIntersection};
}

//! Creates a vertical edge from the start point of one edge downward to the edge below, and splits the polygon,
//! creating a new region on the twin side of the vertical edge. Returns the twin edge (the edge on the new region)
//! If there is already a vertical edge between the start point and the intersection point below it, then no new edge
//! is created, and the existing one is returned.
dcel::HalfEdge startLowerRegion(PolygonEdge* edgeAbove, PolygonEdge* edgeBelow, VerticalEdgeList& verticalEdges, Dcel& dcel)
{
    // The edge below should have a half edge which goes
    // clockwise on the end result region, so we will split it apart
    // if the intersection is not where it ends
    // and point it to the twin half edge coming back up
    ASSERT(edgeBelow->getData<EdgeData>()->halfEdge());

    // The edge above should not have a half edge yet, because this
    // should be the first time it is being processed
    ASSERT(!edgeAbove->getData<EdgeData>()->halfEdge());

    // Assumption, there are no edges to the right of edgeBelow added to the
    // dcel, so going backwards one step will move to the half-edge that connects
    // the top of the region to the bottom; the 'right' side of the region
    const auto rightRegionEdge = edgeBelow->getData<EdgeData>()->halfEdge().previous();

    // The edge will go to/from this point, which should
    // be the tip of a hole in the overall shape
    const auto pointAbove = edgeAbove->getData<EdgeData>()->firstVertex();

    edgeBelow = adjustEdgeForIntersectionBelow(pointAbove, edgeBelow);

    // Get the vertex object for where the vertical edge to be created
    // intersects with the edge below. If the point is not in the dcel yet,
    // it will be added.
    const auto intersectionVertex = intersectionBelow(pointAbove, edgeBelow->getData<EdgeData>()->halfEdge(), dcel);

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

//! Similar to startLowerRegion, except that this should be used on the closing side of the inner loop to end
//! the region that was created below it. This creates a vertical split from the end of the loop down to the edge
//! below, and starts a new region on the twin side of the edge.
//! If the edge already exists, then no new edge is created, and the existing one is returned.
dcel::HalfEdge endLowerRegion(PolygonEdge* edgeAbove, PolygonEdge* edgeBelow, VerticalEdgeList& verticalEdges, Dcel& dcel)
{
    // Both edges should have been processed previously, so both
    // should already have a half edge assigned
    ASSERT(edgeAbove->getData<EdgeData>()->halfEdge());
    ASSERT(edgeBelow->getData<EdgeData>()->halfEdge());
    ASSERT(!edgeAbove->isVertical());

    // The edge will go to/from this point, which should
    // be the tip of a hole in the overall shape
    const auto pointAbove = edgeAbove->getData<EdgeData>()->secondVertex();

    edgeBelow = adjustEdgeForIntersectionBelow(pointAbove, edgeBelow);

    // Get the vertex object for where the vertical edge to be created
    // intersects with the edge below. If the point is not in the dcel yet,
    // it will be added.
    const auto intersectionVertex = intersectionBelow(pointAbove, edgeBelow->getData<EdgeData>()->halfEdge(), dcel);

    // Check if the edge exists already; this could happen if there are multiple holes
    // in a shape which all line up where they start
    const auto existingEdge = verticalEdges.getEdge(pointAbove, intersectionVertex);
    if (existingEdge)
        return existingEdge.twin();

    dcel::HalfEdge edgeBeforeIntersection, edgeAfterIntersection;
    std::tie(edgeBeforeIntersection, edgeAfterIntersection) = getEdgesBeforeAndAfterIntersection(edgeBelow, intersectionVertex, dcel);

    // Split the region by drawing an edge straight down from the opening point of the
    // hole. If there's already an edge there, then this will just add a twin to it.
    const auto downwardHalfEdge = dcel.splitRegion(edgeAbove->getData<EdgeData>()->halfEdge(), edgeAfterIntersection);

    verticalEdges.insert(downwardHalfEdge);

    return downwardHalfEdge.twin();
}

//! Creates a vertical edge from the start point of one edge upward to the edge above, and splits the polygon,
//! creating a new region on the twin side of the vertical edge. Returns the twin edge (the edge on the new region)
//! This edge should never exist, so if it does, and AssertionError is thrown.
//! It is expected that startLowerRegion() will have already been called, and therefore a vertical edge will exist to be
//! extended via this newly created edge.
dcel::HalfEdge startUpperRegion(PolygonEdge* edgeAbove, dcel::HalfEdge verticalDownwardEdgeBelow, VerticalEdgeList& verticalEdges,
                                Dcel& dcel)
{
    // The edge above should have a half edge which goes
    // clockwise on its region, so we will split it apart
    // if the intersection is not where it ends
    // and point it to the twin half edge coming back down
    ASSERT(edgeAbove->getData<EdgeData>()->halfEdge());

    const auto pointBelow = verticalDownwardEdgeBelow.origin();

    edgeAbove = adjustEdgeForIntersectionAbove(pointBelow, edgeAbove);

    const auto intersectionVertex = intersectionAbove(pointBelow, edgeAbove->getData<EdgeData>()->halfEdge(), dcel);
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
    edgeAbove->getData<EdgeData>()->halfEdge() = edgeAfterIntersection;

    // Create an edge going from the end of the edge before the intersection
    // to the beginning of the edge we used to know where to draw upwards.
    // This will close the region, and create a new region on the other side of
    // this new edge
    const auto downwardHalfEdge = dcel.splitRegion(edgeBeforeIntersection, verticalDownwardEdgeBelow);

    verticalEdges.insert(downwardHalfEdge);

    // Return the twin, which is the one going upwards
    return downwardHalfEdge.twin();
}

//! Creates a vertical edge from the end point of one edge upward to the edge above and ends the region that
//! is to the left of this edge. It is expected that this region will be the region created above some internal
//! loop in the overal shape.
//! If the edge already exists, then the existing edge will be returned.
//! After the region is closed and a new one is created on the opposite side of the new vertical line,
//! that new region will be combined with the region directly below it, as created by endLowerRegion(); this
//! results in a single region as the sweep line continues to the right after encountering the end of
//! an internal loop.
dcel::HalfEdge endUpperRegion(PolygonEdge* edgeAbove, PolygonEdge* edgeBelow, dcel::HalfEdge verticalUpwardEdgeBelow,
                              VerticalEdgeList& verticalEdges, Dcel& dcel)
{
    // The edge above should have a half edge which goes
    // clockwise on its region, so we will split it apart
    // if the intersection is not where it ends
    // and point it to the twin half edge coming back down
    ASSERT(edgeAbove->getData<EdgeData>()->halfEdge());
    ASSERT(edgeBelow->getData<EdgeData>()->halfEdge());
    ASSERT(edgeBelow->getData<EdgeData>()->secondVertex() == verticalUpwardEdgeBelow.next().origin());

    const auto pointBelow = verticalUpwardEdgeBelow.next().origin();

    edgeAbove = adjustEdgeForIntersectionAbove(pointBelow, edgeAbove);

    const auto intersectionVertex = intersectionAbove(pointBelow, edgeAbove->getData<EdgeData>()->halfEdge(), dcel);
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
    edgeAbove->getData<EdgeData>()->halfEdge() = edgeAfterIntersection;

    // Create an edge going from the end of the edge before the intersection
    // to the beginning of the edge we used to know where to draw upwards.
    // This will close the region, and create a new region on the other side of
    // this new edge
    const auto downwardHalfEdge = dcel.splitRegion(edgeBeforeIntersection, edgeBelow->getData<EdgeData>()->halfEdge());
    verticalEdges.insert(downwardHalfEdge);

    // At this point, there's two regions on the righthand side of the vertical split; but there
    // shouldn't be in the end result - we want those regions to be combined
    const auto upwardHalfEdge = downwardHalfEdge.twin();

    dcel.mergeRegions(verticalUpwardEdgeBelow, upwardHalfEdge, upwardHalfEdge.previous(), verticalUpwardEdgeBelow.next().next());

    // Return the twin, which is the one going upwards
    return upwardHalfEdge;
}

//! Calculates the angle between segments AB and BC, given points A, B, and C
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
}
}
}
}
