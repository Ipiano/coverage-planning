#include "vertical-edge-list.h"

namespace ads
{
namespace ccpp
{
namespace polygon_decomposer
{
namespace modified_trapezoidal
{
void VerticalEdgeList::insert(dcel::HalfEdge edge)
{
    ASSERT(edge && edge.twin());
    ASSERT(std::abs(edge.origin().point().x() - edge.twin().origin().point().x()) < epsilon);
    ASSERT(edge.origin().point().y() >= edge.twin().origin().point().y());

    const auto insertIt = std::lower_bound(m_edges.begin(), m_edges.end(), edge, VerticalEdgeLessThan());
    m_edges.insert(insertIt, edge);
}

dcel::HalfEdge VerticalEdgeList::getEdge(const dcel::Vertex v1, const dcel::Vertex v2)
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
}
}
}
}
