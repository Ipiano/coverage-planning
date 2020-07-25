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

class VerticalEdgeList
{
    std::vector<dcel::HalfEdge> m_edges;

    struct VerticalEdgeLessThan
    {
        bool operator()(const dcel::HalfEdge e, const dcel::Vertex v) { return e.origin().point().x() < v.point().x(); }
        bool operator()(const dcel::Vertex v, const dcel::HalfEdge e) { return v.point().x() < e.origin().point().x(); }
        bool operator()(const dcel::HalfEdge l, const dcel::HalfEdge r) { return operator()(l, r.origin()); }
    };

  public:
    // Should always insert the downward edge
    void insert(dcel::HalfEdge edge);

    dcel::HalfEdge getEdge(const dcel::Vertex v1, const dcel::Vertex v2);
};
}
}
}
}
