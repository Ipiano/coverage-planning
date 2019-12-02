#pragma once

#include "ads/ccpp/typedefs.h"

#include <boost/geometry/index/rtree.hpp>

#include <memory>

namespace ads
{
namespace ccpp
{
namespace dcel
{
struct half_edge_t;
struct const_half_edge_t;

struct vertex_t
{
    half_edge_t* edge = nullptr;
    geometry::Point2d location;

    operator geometry::Point2d&(){ return location; }
    operator const geometry::Point2d&() const { return location; }
};

struct const_vertex_t
{
    const const_half_edge_t* edge = nullptr;
    const geometry::Point2d location;

    operator const geometry::Point2d&() const { return location; }
};

struct face_t
{
    half_edge_t* edge = nullptr;
};

struct const_face_t
{
    const const_half_edge_t* edge = nullptr;
};

struct half_edge_t
{
    half_edge_t* twin = nullptr;
    half_edge_t* next = nullptr;
    half_edge_t* prev = nullptr;
    vertex_t* origin = nullptr;
    face_t* face = nullptr;
};

struct const_half_edge_t
{
    const const_half_edge_t* twin = nullptr;
    const const_half_edge_t* next = nullptr;
    const const_half_edge_t* prev = nullptr;
    const const_vertex_t* origin = nullptr;
    const const_face_t* face = nullptr;
};

struct vertex_index
{
    typedef geometry::Point2d result_type;

    const result_type& operator() (const std::shared_ptr<vertex_t>& v) const
    {
        return v->location;
    }
};

}


class DoublyConnectedEdgeList
{
    boost::geometry::index::rtree<
            std::shared_ptr<dcel::vertex_t>,
            boost::geometry::index::quadratic<100>,
            dcel::vertex_index
        > m_vertices;

    std::vector<std::shared_ptr<dcel::face_t>> m_faces;
    std::vector<std::shared_ptr<dcel::half_edge_t>> m_edges;

    double m_epsilon;

    dcel::vertex_t* find(const geometry::Point2d&) const;
    dcel::vertex_t* findOrCreate(const geometry::Point2d&);
    void addLoop(const geometry::Ring2d&ring, dcel::face_t* rightFace, dcel::face_t* leftFace);

public:
    DoublyConnectedEdgeList(const geometry::Polygon2d& initialShape, const double epsilon = 0.00001);

    DoublyConnectedEdgeList(const DoublyConnectedEdgeList&) = delete;
    DoublyConnectedEdgeList(DoublyConnectedEdgeList&&);

    ~DoublyConnectedEdgeList() = default;

    const dcel::const_face_t* insideFace() const;
    std::vector<const dcel::const_half_edge_t*> edges(const dcel::const_face_t *face) const;
};

}
}
