#include "edge-data.h"

namespace ads
{

using namespace algorithms::sweep_line;
using namespace dcel;

namespace ccpp
{
namespace polygon_decomposer
{
namespace modified_trapezoidal
{
EdgeData::EdgeData(PolygonEdge* edge, Dcel& dcel)
    : m_edge(edge), m_firstVertex(dcel.vertex(edge->firstPoint())), m_secondVertex(dcel.vertex(edge->secondPoint())), m_processed(false)
{
}

// Gets the points of the edge connected to 'leftEdge()' and 'rightEdge()' respectively
dcel::Vertex EdgeData::leftVertex() const
{
    return firstVertex() == leftEdge()->getData<EdgeData>()->firstVertex() ||
                   firstVertex() == leftEdge()->getData<EdgeData>()->secondVertex()
               ? firstVertex()
               : secondVertex();
}
dcel::Vertex EdgeData::rightVertex() const
{
    return firstVertex() == rightEdge()->getData<EdgeData>()->firstVertex() ||
                   firstVertex() == rightEdge()->getData<EdgeData>()->secondVertex()
               ? firstVertex()
               : secondVertex();
}

PolygonEdge* EdgeData::leftEdge() const
{
    return !edge()->isVerticalZigZag()
               ? edge()->firstEdge()
               : (edge()->firstEdge()->getData<EdgeData>()->firstVertex() == firstVertex() ? edge()->secondEdge() : edge()->firstEdge());
}

PolygonEdge* EdgeData::rightEdge() const
{
    return !edge()->isVerticalZigZag()
               ? edge()->secondEdge()
               : (edge()->firstEdge()->getData<EdgeData>()->firstVertex() == firstVertex() ? edge()->firstEdge() : edge()->secondEdge());
}

// Always finds the 'bottom' side
// of a loop start, to guarantee it's
// found asap
bool EdgeData::isLoopStart() const
{
    return !edge()->isVertical() && !edge()->firstEdge()->isVerticalZigZag() &&
           firstVertex() == edge()->firstEdge()->getData<EdgeData>()->firstVertex() && edge()->angleLessThan(edge()->firstEdge());
}

// Not guaranteed to find any
// particular top or bottom of an
// loop, but always guarantees that it's
// the last edge on the loop processed
bool EdgeData::isLoopEnd() const
{
    return edge()->nextEdge()->getData<EdgeData>()->processed() && edge()->previousEdge()->getData<EdgeData>()->processed();
}
}
}
}
}
