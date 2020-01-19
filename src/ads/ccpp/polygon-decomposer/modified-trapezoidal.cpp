#include "modified-trapezoidal.h"

#include "ads/ccpp/sort-edges.h"

#include <boost/geometry/strategies/transform.hpp>

#include <algorithm>
#include <vector>

namespace bg = boost::geometry;

namespace ads
{
namespace ccpp
{
namespace polygon_decomposer
{

void ModifiedTrapezoidal::decompose(std::vector<const dcel::const_half_edge_t*>& sortedEdges, DoublyConnectedEdgeList& dcel) const
{
    // Tracks which edges have been processed
    std::unordered_map<const dcel::const_half_edge_t*, bool> processed;

    //! 1. Initialize active edge list to empty
    // These are edges that are 'in scope' to be considered
    // std::vector<EdgeType> activeEdges;

    //! 2. Initialize list of edges to be completed to empty
    //  These are edges that have been seen, but not added to the result?
    // std::vector<EdgeType> newEdges;

    //! 3. Initialize the location of the last vertical to "not seen"
    //  This tracks where the last line that split out a new shape was
    double lastVertical = std::numeric_limits<double>::lowest();

    // Tracks the sweep line
    const dcel::const_vertex_t* sweepLineLocation;

    //! 4. For each segment in the sorted input list
    for (const auto& edge : sortedEdges)
    {
        //! a. Mark the edge as processed
        processed[edge] = true;

        //! b. Update the location of the sweep line to the 'left' endpoint
        //  Should we rearrange the segments such that this is always the first
        //  one? What was the 'segment' type in use for this paper?
        sweepLineLocation = edge->origin;

        //! c. If the list of new edges to be completed is not empty and the
        //! location of the new edge list falls before the left endpoint of the
        //! current edge, find the index in the active edge list of the edge the
        //! vertical comes off of, and connect the vertical to the edges immediately
        //! above and below it in the active edge list
        //  What is the 'location' of the new edge list? Is it the location of
        //  the leftmost point of the first segment? If segments are only inserted
        //  into it the order the are processed in this loop, then the first segment
        //  is guaranteed to be the leftmost one.
        //
        //  What is the edge that the vertical comes off of? What is the vertical?
        //  Is it talking about the previous one, or the one we're about to create?
        //
        //  Once we connect to the edges above and below in the active edge list, then
        //  what? Do we remove them and that forms a new polygon?

        //! d. Update the active edge list, adding the new edge, and removing any
        //! edges that have gone out of scope
        //  What is 'out of scope'? Is that any edges s.t. the rightmost point is
        //  to the left of the current edge's leftmost point? That should be
        //  very few segments, because it would imply that all of their endpoints
        //  are on the same y coordinate, which is unlikely in the real world, so
        //  usual case will be that only the edge directly connected to the one
        //  being examined would be removed from the list

        //! e. If neither the previous nor the next edge along the boundary has been
        //! encountered, and the current edge is currently an "upper" edge (i.e. its
        //! position in the active edge list is divisible by 2), start an exterior
        //! boundary
        //  Define 'encountered'? Does that involve the 'processed' list?
        //
        //  Being able to get the next edge along the boundary implies that segments
        //  need to have a 'next' member, or link back to the original shape or something.
        //  But what is 'along the boundary' mean, which direction do you go?
        //
        //  What does being divisible by 2 have to do with anything? I can see that making
        //  sense if you are guaranteed to have the same number of segments on the 'top'
        //  of a shape as the 'bottom', but that's not real. Is it an "upper" edge if
        //  top part of the shape has a weird hook and starts coming back in on itself?
        //
        //  This section is essentially "If we hit the leftmost point on an exterior loop,
        //  start tracking it as such"

        //! i. Get the next segment in the sorted input list. This represents the second
        //! edge of the boundary.

        //! ii. Construct a half-edge for it, storing the new half edge as a duplicate
        //! of the original edge
        // What is 'it' in this context? The next segment probably. What is the original
        // edge? 'it'? Or the edge we're examining in the loop through the sorted edges?

        //! iii. Update the active edge list
        //  Yup this is obvious what I'm supposed to do here

        //! iv. Connect the two edges together
        //  Which two edges? The segment from the loop and the next segment on the polygon?
        //  Those are already connected. The new half-edge and the segment we created it from?
        //  That doesn't make sense unless we're re-using the sorted input as part of the DCEL output.

        //! f. Else, if neither the previous nor the next edge along the boundary has been processed yet,
        //! start an interior boundary.
        //  Essentially "If we hit the leftmost point on an interior loop, start tracking it as such"

        //! i. Get the next segment in the sorted input list. This represents the second edge of the boundary
        //  Why is the guaranteed?? There's NO reason it has to be anywhere near the one we're currently
        //  looking at. Were there assumptions made about the input shape somewhere??

        //! ii. Mark the second edge as processed
        //  processed[nextEdge] = true

        //! iii. Update the active edge list
        //  Ah yes, let me just do an 'update'

        //! iv. Connect the two edges together

        //! v. Close the current region and start two new regions. Construct an
        //! edge connecting the left end point of the first edge on the interior boundary
        //! to the upper exterior boundary (the edge in the active edge list immediately before
        //! the first interior edge). Construct an edge connecting the left end point of the first edge
        //! on the interior boundary to the lower exterior boundary (the edge in the active edge list
        //! immediately after the second interior edge). The two edges created in this step should form
        //! a straight line.
        //  Simply put, make a vertical line at the leftmost point of the interior shape, and
        //  close off the regions that it's going to be part of. But this talks about extending it on
        //  both sides to the exterior boundary; what if there's another interior loop above or below it?
        //  Maybe that's accounted for the the condition stated in '4.f'?
        //
        //  See 4.g.ii, which is the process for creating the vertical line at the end of this loop. That
        //  section puts the unfinished vertical pieces into the new edges list because it's not known where
        //  they will connect; maybe we need to do that here too?

        //! g. Else if the current edge is the last edge to be processed on an interior boundary (i.e. both the
        //! previous and next edges have already been processed and the current edge isn't the first or last edge in the
        //! active edge list)
        //  What are the rules for the active edge list? How could the current edge not be the first or last item? Don't
        //  we always insert stuff at the end of the list every time?
        //
        //  Basically, make a vertical line whenever we close an interior shape

        //! i. Connect up the edge to the boundary, creating a new edge at the sweep location if necessary. Details of
        //! this step are given in step i. below
        // Step 4.h.i below, maybe?
        // Why is 'boundary' used to describe every single goddamn shape in this algorithm.

        //! ii. Close two regions and start a new region. Construct two edges, one from the right end point to the upper
        //! exterior boundary, and one from the right end point of the current edge to the lower exterior boundary,
        //! connecting the two edges together. Leave the edges unconnected to the upper and lower exterior boundaries as it
        //! is not yet known which edges it would be connected to at the top and bottom. Add the new edges to the list of new
        //! edges to be completed
        //  Well now I know what the new edges list is for. Why do we use that here, but not at the start of the interior loop??

        //! h. Else if the current edge is the last edge to be processed on the exterior boundary
        //  Should be the last line? If we assume there's a single exterior loop with multiple
        //  interior loops, this should be the last segment processed.

        //! i. Build a new edge at the location of the sweep line, connecting it to the previous edge
        //! in the active edge list (the upper exterior boundary), and to the next edge in the active edge list
        //! (the lower exterior boundary).
        //  Maybe active edges aren't edges that are just 'in scope', but specifically edges on the exterior
        //  loop?

        //! ii. Connect up the edge to the previous or next edge on the boundary, as appropriate, so as to close
        //! the exterior boundary

        //! i. Else
        //  Gotta love numbering schemes

        //! i. If the sweep line is more than the pass-width from the previous dividing line, or if an edge created at the location
        //! of the current sweep line would fall outside the field, or if the angle between the current edge and the edge to the left
        //! is close to 180 degrees, connect up the edge to the boundary, without closing the region
        //  This is the 'modified' part of the trapizoidal algorithm. Normally you would close the region at every single point
        //  enountered, but there's so many points and they're likely approximately linear, so we just keep adding them to the side
        //  of the shape
        //
        //  For the purposes of the paper, 'close to 180 degrees' was +/- 10

        //! ii. Else close the current region and start a new one. Construct a new edge parallel to the sweep line, connecting
        //! the left end point of the current edge to the previous edge in the active edge list (if the current edge is a 'lower' edge)
        //! or to the next edge in the active edge list (if the current edge is an 'upper' edge)
    }
}

void ModifiedTrapezoidal::decomposePolygon(DoublyConnectedEdgeList& dcel) const
{
    auto edges = dcel.edges(dcel.insideFace());

    sortEdges(edges, m_sweepDir);

    decompose(edges, dcel);
}
}
}
}
