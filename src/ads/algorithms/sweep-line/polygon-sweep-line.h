#pragma once

#include "ads/algorithms/sweep-line/geometry.h"
#include "ads/algorithms/sweep-line/edge.h"
#include "ads/algorithms/sweep-line/active-edges-list.h"

#include <vector>
#include <memory>

namespace ads
{
namespace algorithms
{
namespace sweep_line
{
/*!
 * \brief Base sweep-line algorithm for working with polygon/loop data
 *
 * The sweep line algorithm is a common geometric algorithm used to process all of the
 * line segments in a set in order from one side to the other. Points are sorted by either X or
 * Y coordinate, and then processed in that sorted order. When a line segment is first encountered,
 * it is added to the set of 'active' edges, and when the second point on the edge is encountered,
 * it is removed. If the edges are sorted by X coordinate, this gives the ability to
 * quickly find all edges directly above or below the current point - the set of edges that would
 * be intersected by a vertical line through the active point.
 *
 * This class is a specialized version which is expected to be used when the data being processed
 * forms one or more closed loops which may be nested or treated as holes.
 *
 * PolygonSweepLineAlgorithm manages ordering the edges and tracking the set of which ones are 'active'.
 * Each instance of the algorithm can be run exactly one time. When run, it will build the
 * sorted list of edges and traverse it. Subclasses can override the 'sweepStep' method to
 * process all the edges that were added to the 'active' set at a location. When the end of
 * the list is reached, the 'sweepEnd' method will be called.
 *
 * PolygonSweepLineAlgorithm sorts first by X coordinate, then by Y. Any edges which share their first
 * point are sorted by angle, with the edge that angles downward sorted first.
 *
 * The notable exception to this sorting method is when there is a zig-zag like this example
 *
 * \verbatim
  *-a-*
      |
      b
      |
      *-c-*
   \endverbatim

 * according to the rules above, the edges should be ordered
 *
 * a -> c -> b. However, they will be ordered a -> b -> c as it is often useful
 * to process edges in the order they are connected around a polygon
 *
 * At every step of the algorithm, the set of active edges is guaranteed to have an even
 * number of edges. This invariant is maintained even when there are vertical edges, as those
 * edges are never added to the 'active' edges set.
 */
class PolygonSweepLineAlgorithm
{
  public:
    typedef std::vector<std::unique_ptr<PolygonEdge>> EdgeList;

    PolygonSweepLineAlgorithm();

    /*!
     * \brief Runs the sweep line algorithm on a polygon
     *
     * Can only be called once. Returns false after the first call.
     *
     * During the algorithm execution, sweepBegin() will be called once, followed
     * by sweepStep() zero or more times, followed by sweepEnd(). Any references
     * to data passed to these methods are valid until sweepEnd() returns.
     *
     * \param polygon Polygon to process
     * \return True if this is the first time running and the algorithm runs
     */
    bool run(const Polygon2d& polygon);

  protected:
    virtual ~PolygonSweepLineAlgorithm();

    /*!
     * \brief Called once at the beginning of the sweep, after the edges are sorted
     *
     * This call can be used to initialize the custom data on each edge using
     * Edge::setData<T>
     *
     * \param edgesBegin Begin iterator to sorted edge list
     * \param edgesEnd After-end iterator to sorted edge list
     */
    virtual void sweepBegin(typename EdgeList::const_iterator edgesBegin, typename EdgeList::const_iterator edgesEnd)
    {
        boost::ignore_unused(edgesBegin);
        boost::ignore_unused(edgesEnd);
    }

    /*!
     * \brief Called during every step of the sweep algorithm
     *
     * A 'step' involves moving the sweep line to the next edge
     * in the sorted edge list, removing all edges to the left of the
     * new sweep line location, and inserting all edges which begin on
     * the X coordinate of the point that was stepped to.
     *
     * Once the active edges list is updated, sweepStep() will be called with
     * iterators to the first edge in the step, and the edge after the last one
     * in the step.
     *
     * \param stepBegin Iterator to first edge in the step
     * \param stepEnd Iterator after the last edge in the step
     * \param activeEdges Active edges list after updating for this step
     */
    virtual void sweepStep(typename EdgeList::const_iterator stepBegin, typename EdgeList::const_iterator stepEnd,
                           const ActivePolygonEdgesList& activeEdges)
    {
        boost::ignore_unused(stepBegin);
        boost::ignore_unused(stepEnd);
        boost::ignore_unused(activeEdges);
    }

    /*!
     * \brief Called once after the sweep line algorithm is completed
     *
     * All references stored to sweep line data during sweepBegin() and sweepStep() are invalidated
     * after this call returns.
     *
     * \param stepBegin Iterator to the first edge in the sorted list
     * \param stepEnd Iterator to the last edge in the sorted list
     */
    virtual void sweepEnd(typename EdgeList::const_iterator edgesBegin, typename EdgeList::const_iterator edgesEnd)
    {
        boost::ignore_unused(edgesBegin);
        boost::ignore_unused(edgesEnd);
    }

  private:
    bool m_ran;

    std::vector<std::unique_ptr<PolygonEdge>> static buildSortedEdgeList(const Polygon2d& poly);
};
}
using sweep_line::PolygonSweepLineAlgorithm;
}
}
