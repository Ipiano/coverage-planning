#pragma once

#include "ads/dcel/geometry.h"
#include "ads/dcel/handles.h"

namespace ads
{
namespace dcel
{

class Dcel;

template <class HandleT> class Object
{
    friend class Dcel;

    const Dcel* m_dcel;
    HandleT m_handle;

  protected:
    Object(const Dcel* dcel, HandleT handle) : m_dcel(dcel), m_handle(handle) {}

  public:
    const HandleT handle() const { return m_handle; }
    const Dcel* dcel() const { return m_dcel; }
};

class HalfEdge;
class Vertex : public Object<VertexHandle>
{
    friend class Dcel;
    friend class HalfEdge;

    Vertex(const Dcel* dcel, VertexHandle handle);

  public:
    Vertex() : Vertex(nullptr, NoVertex) {}

    operator bool() const;
    bool operator==(const Vertex& other) const;
    bool operator!=(const Vertex& other) const;

    Point2d point() const;
    HalfEdge edge() const;
};

class Region : public Object<RegionHandle>
{
    friend class Dcel;

    Region(const Dcel* dcel, RegionHandle handle);

  public:
    Region() : Region(nullptr, NoRegion) {}

    operator bool() const;
    bool operator==(const Region& other) const;
    bool operator!=(const Region& other) const;

    HalfEdge edge() const;

    template <class Functor> Functor forEachSegment(Functor f) const;
};

class HalfEdge : public Object<HalfEdgeHandle>
{
    friend class Dcel;

    HalfEdge(const Dcel* dcel, HalfEdgeHandle handle);

  public:
    HalfEdge() : HalfEdge(nullptr, NoHalfEdge) {}

    operator bool() const;
    bool operator==(const HalfEdge& other) const;
    bool operator!=(const HalfEdge& other) const;

    HalfEdge next() const;
    HalfEdge previous() const;
    HalfEdge twin() const;

    Vertex origin() const;
    Region region() const;
};

namespace hash
{
//! Hash struct for dcel objects so they can be used as map keys
template <class ObjectT> struct Object
{
    std::size_t operator()(const ObjectT& h) const { return std::hash<unsigned long long>()(h.handle().t); }
};
}
}
}

namespace std
{
template <> struct hash<ads::dcel::Region> : public ads::dcel::hash::Object<ads::dcel::Region>
{
};

template <> struct hash<ads::dcel::HalfEdge> : public ads::dcel::hash::Object<ads::dcel::HalfEdge>
{
};

template <> struct hash<ads::dcel::Vertex> : public ads::dcel::hash::Object<ads::dcel::Vertex>
{
};
}
