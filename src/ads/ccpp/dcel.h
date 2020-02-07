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

struct region_t
{
    half_edge_t* edge = nullptr;
};

struct half_edge_t
{
    half_edge_t* twin = nullptr;
    half_edge_t* next = nullptr;
    half_edge_t* prev = nullptr;
    vertex_t* origin  = nullptr;
    region_t* region  = nullptr;
};
}

struct DoublyConnectedEdgeList
{
    std::vector<std::unique_ptr<dcel::region_t>> regions;
    std::vector<std::unique_ptr<dcel::half_edge_t>> edges;
    std::vector<std::unique_ptr<dcel::vertex_t>> vertices;
};
}
}
