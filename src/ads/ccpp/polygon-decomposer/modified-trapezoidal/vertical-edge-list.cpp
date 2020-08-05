#include "vertical-edge-list.h"

#include "ads/algorithms/sweep-line/geometry.h"

namespace ads
{
namespace ccpp
{
namespace polygon_decomposer
{
namespace modified_trapezoidal
{

struct VerticalEdgeLessThan
{
    /*!
     * \brief Comparator for two edges in the vertical edges list
     *
     * Since we guarantee that the inserted edges are vertical such that
     * the origin of the half edge stored is the top point, we just check if
     * the bottom point compares < the top point of the other. If they are on
     * the same X coordinate, this will check if one is below the other; otherwise
     * it will sort by the X coord.
     *
     * \param l First half edge
     * \param r Second half edge
     * \return True if they should be sorted l before r
     */
    bool operator()(const dcel::HalfEdge l, const dcel::HalfEdge r)
    {
        return algorithms::sweep_line::pointLessThan(l.next().origin().point(), r.origin().point());
    }
};

void VerticalEdgeList::insert(dcel::HalfEdge edge)
{
    ASSERT(algorithms::sweep_line::haveSameXCoord(edge.origin().point(), edge.twin().origin().point()));
    ASSERT(edge.origin().point().y() >= edge.next().origin().point().y());

    const auto insertIt = std::lower_bound(m_edges.begin(), m_edges.end(), edge, VerticalEdgeLessThan());
    m_edges.insert(insertIt, edge);
}

dcel::HalfEdge VerticalEdgeList::getEdge(const dcel::Vertex v1, const dcel::Vertex v2)
{
    auto firstIt      = std::lower_bound(m_edges.begin(), m_edges.end(), v1, VerticalEdgeLessThan());
    const auto lastIt = std::upper_bound(m_edges.begin(), m_edges.end(), v1, VerticalEdgeLessThan());

    for (; firstIt != lastIt; firstIt++)
    {
        if ((firstIt->origin() == v1 && firstIt->next().origin() == v2) ||
            (firstIt->next().origin() == v2 && firstIt->next().origin() == v1))
            return *firstIt;
    }

    return dcel::HalfEdge();
}
}
}
}
}
