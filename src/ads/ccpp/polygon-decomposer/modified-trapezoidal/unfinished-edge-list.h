#pragma once

#include "ads/algorithms/sweep-line/edge.h"

#include <vector>
#include <algorithm>

namespace ads
{
namespace ccpp
{
namespace polygon_decomposer
{
namespace modified_trapezoidal
{

/*!
 * \brief Tracks edges identified during the modified trapezoidal decomposition as vertical split locations that cannot be split yet
 *
 * As the sweep line moves across the shape, locations will be identified where a region
 * needs to be created by adding a vertical line to split the shape. However, if the
 * sweep line has not progressed past the point identified, it is not possible to split the shape.
 *
 * This list will track the edges that have been identified as needing to be a split location
 * but which could not have a split created. It supports a queue-like behavior, where they
 * are sorted by the order that the sweep line will reach them and can be removed in that
 * order.
 */
class UnfinishedEdgeList
{
    std::vector<algorithms::sweep_line::PolygonEdge*> m_edges;

  public:
    //! Gets the number of eges stored
    size_t size() const;

    //! Adds a polygon edge to the list
    void insert(algorithms::sweep_line::PolygonEdge* e);

    //! Gets the next edge in the list
    algorithms::sweep_line::PolygonEdge* next() const;

    //! Gets the last edge in the list
    algorithms::sweep_line::PolygonEdge* last() const;

    //! Removes the current next edge in the list
    //! This list must not be empty when this is called
    void pop();
};
}
}
}
}
