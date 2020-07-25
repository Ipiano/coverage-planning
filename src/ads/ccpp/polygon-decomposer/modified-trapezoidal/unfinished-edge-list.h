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
// List of unfinished edges sorted in sweep line order
// by the second point on the segments
class UnfinishedEdgeList
{
    std::vector<algorithms::sweep_line::PolygonEdge*> m_edges;

  public:
    size_t size() const;
    void insert(algorithms::sweep_line::PolygonEdge* e);

    algorithms::sweep_line::PolygonEdge* next() const;
    algorithms::sweep_line::PolygonEdge* last() const;
    void pop();
};
}
}
}
}
