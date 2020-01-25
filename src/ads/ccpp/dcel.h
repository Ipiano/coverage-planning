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

struct vertex_t
{
    half_edge_t* edge = nullptr;
    geometry::Point2d location;

    operator geometry::Point2d&() { return location; }
    operator const geometry::Point2d&() const { return location; }
};

struct face_t
{
    half_edge_t* edge = nullptr;
};

struct half_edge_t
{
    half_edge_t* twin = nullptr;
    half_edge_t* next = nullptr;
    half_edge_t* prev = nullptr;
    vertex_t* origin  = nullptr;
    face_t* face      = nullptr;
};
}

struct DoublyConnectedEdgeList
{
    std::vector<std::shared_ptr<dcel::face_t>> faces;
    std::vector<std::shared_ptr<dcel::half_edge_t>> edges;
    std::vector<std::shared_ptr<dcel::vertex_t>> vertices;
};
}
}
