#include "unfinished-edge-list.h"

namespace ads
{
namespace ccpp
{
namespace polygon_decomposer
{
namespace modified_trapezoidal
{
size_t UnfinishedEdgeList::size() const
{
    return m_edges.size();
}
void UnfinishedEdgeList::insert(algorithms::sweep_line::PolygonEdge* e)
{
    const auto insertIt =
        std::lower_bound(m_edges.begin(), m_edges.end(), e,
                         [](const algorithms::sweep_line::PolygonEdge* e1, const algorithms::sweep_line::PolygonEdge* e2) {
                             return algorithms::sweep_line::pointLessThan(e1->secondPoint(), e2->secondPoint());
                         });

    m_edges.insert(insertIt, e);
}

algorithms::sweep_line::PolygonEdge* UnfinishedEdgeList::next() const
{
    return m_edges.front();
}

algorithms::sweep_line::PolygonEdge* UnfinishedEdgeList::last() const
{
    return m_edges.back();
}

void UnfinishedEdgeList::pop()
{
    m_edges.erase(m_edges.begin());
}
};
}
}
}
