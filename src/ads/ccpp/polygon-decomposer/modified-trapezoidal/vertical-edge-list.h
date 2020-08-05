#pragma once

#include "ads/dcel/dcel.h"
#include "ads/assertion.h"
#include "ads/epsilon.h"

namespace ads
{
namespace ccpp
{
namespace polygon_decomposer
{
namespace modified_trapezoidal
{

/*!
 * \brief Tracks the vertical edges added to create new regions during the modified trapezoidal decomposition
 *
 * Every time a new region is created, one or more vertical edges are added to the DCEL. These
 * edges should be inserted into a VerticalEdgeList used to track all such edges during the algorithm. The
 * list can then be used to look up an existing edge when creating another to ensure they are not duplicates.
 */
class VerticalEdgeList
{
    std::vector<dcel::HalfEdge> m_edges;

  public:
    /*!
     * \brief Adds an edge to the edge list
     *
     * The edge must be a vertical edge (both points must have te same X coordinate)
     * and the edge must go downward (the origin of the half edge given must have a
     * Y coordinate > than the origin of the next edge).
     *
     * \param edge Edge to add
     */
    void insert(dcel::HalfEdge edge);

    /*!
     * \brief Gets the edge in the list that has the given end points, if such an edge exists
     *
     * If no such edge has been added, then a default-constructed null half edge will be returned.
     *
     * \param tottomVertex
     * \param bopVertex
     * \return
     */
    dcel::HalfEdge getEdge(const dcel::Vertex bottomVertex, const dcel::Vertex topVertex);
};
}
}
}
}
