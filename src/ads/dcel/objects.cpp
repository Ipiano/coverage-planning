#include "objects.h"

#include "ads/dcel/dcel.h"

namespace ads
{
namespace dcel
{
Vertex::Vertex(const Dcel* dcel, VertexHandle handle) : Object(dcel, handle)
{
}

Vertex::operator bool() const
{
    return dcel() && handle() != NoVertex && dcel()->hasVertex(*this);
}

bool Vertex::operator==(const Vertex& other) const
{
    return dcel() == other.dcel() && handle() == other.handle();
}

bool Vertex::operator!=(const Vertex& other) const
{
    return !operator==(other);
}

Point2d Vertex::point() const
{
    return !dcel() ? Point2d() : dcel()->point(*this);
}

HalfEdge Vertex::edge() const
{
    return !dcel() ? HalfEdge() : dcel()->edge(*this);
}

Region::Region(const Dcel* dcel, RegionHandle handle) : Object(dcel, handle)
{
}

Region::operator bool() const
{
    return dcel() && handle() != NoRegion && dcel()->hasRegion(*this);
}

bool Region::operator==(const Region& other) const
{
    return dcel() == other.dcel() && handle() == other.handle();
}

bool Region::operator!=(const Region& other) const
{
    return !operator==(other);
}

HalfEdge Region::edge() const
{
    return !dcel() ? HalfEdge() : dcel()->edge(*this);
}

HalfEdge::HalfEdge(const Dcel* dcel, HalfEdgeHandle handle) : Object(dcel, handle)
{
}

HalfEdge::operator bool() const
{
    return dcel() && handle() != NoHalfEdge && dcel()->hasEdge(*this);
}

bool HalfEdge::operator==(const HalfEdge& other) const
{
    return dcel() == other.dcel() && handle() == other.handle();
}

bool HalfEdge::operator!=(const HalfEdge& other) const
{
    return !operator==(other);
}

HalfEdge HalfEdge::next() const
{
    return !dcel() ? HalfEdge() : dcel()->next(*this);
}

HalfEdge HalfEdge::previous() const
{
    return !dcel() ? HalfEdge() : dcel()->previous(*this);
}

HalfEdge HalfEdge::twin() const
{
    return !dcel() ? HalfEdge() : dcel()->twin(*this);
}

Vertex HalfEdge::origin() const
{
    return !dcel() ? Vertex() : dcel()->origin(*this);
}

Region HalfEdge::region() const
{
    return !dcel() ? Region() : dcel()->region(*this);
}
}
}
