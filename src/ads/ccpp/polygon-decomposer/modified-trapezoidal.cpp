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
