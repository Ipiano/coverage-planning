#pragma once

#include "ads/algorithms/sweep-line/edge.h"

#include <boost/geometry/algorithms/intersection.hpp>
#include <boost/geometry/geometries/multi_point.hpp>
#include <boost/geometry/geometries/segment.hpp>

#include <vector>

namespace ads
{
namespace algorithms
{
namespace sweep_line
{

/*!
 * \brief Tracks active edges of a polygon sweep line algorithm.
 *
 * Edges are sorted from bottom to top. They are expected not to intersect with each other.
 * The ActiveEdgesList supports a number of removal operations for taking lines out of the
 * list during a polygon sweep algorithm in order to ensure that the active edges set follows
 * around the edge of the polygon in a meaningful order while the sweep line moves.
 *
 */
class ActivePolygonEdgesList
{
    std::vector<PolygonEdge*> m_edges;

  public:
    //! Gets the size of the active edges list
    long size() const;

    //! Inserts an edge into the list if it is not already there
    //! \return The index of the edge in the active edges list
    long insert(PolygonEdge* e);

    //! Removes the edge connected to e which is on the 'left' side (the side with a smaller X coordinate)
    void removeEdgeToLeft(const PolygonEdge* e);

    //! Equivalent to removeLessThan e->firstPoint()
    void removeLessThan(const PolygonEdge* e);

    //! Equivalent to removeLessThan(e); removeEdgeToLeft(e);
    void removeLessThanEqual(const PolygonEdge* e);

    //! Removes all edges with the second point having an X coordinate less than p's
    void removeLessThan(const Point2d& p);

    //! Removes a specific edge from the list
    void removeSpecific(const PolygonEdge* e);

    //! Returns the index of a specific edge if it is present, otherwise -1
    long indexOf(const PolygonEdge* e) const;

    //! Returns the edge in the list which is below edge e. If no edge would be below e, then -1
    //! e is not required to be in the list. If it is not, then the index is returned as if it were inserted first
    long indexBelow(const PolygonEdge* e) const;

    //! Returns the edge from a specfic index in the list. Does not do bounds checking
    PolygonEdge* operator[](const long index) const;
};
}
}
}
