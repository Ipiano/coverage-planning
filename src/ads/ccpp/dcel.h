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

struct DoublyConnectedEdgeList
{
    std::vector<std::unique_ptr<dcel::region_t>> regions;
    std::vector<std::unique_ptr<dcel::half_edge_t>> edges;
    std::vector<std::unique_ptr<dcel::vertex_t>> vertices;
};

// Mimics boost::geometry::for_each_segment
template <class Functor> Functor for_each_segment(const dcel::region_t& region, Functor f)
{
    auto it = region.edge;

    do
    {
        const geometry::ConstReferringSegment2d segment(it->origin->location, it->next->origin->location);
        f(segment);
        it = it->next;
    } while (it != region.edge);

    return f;
}

template <class Functor> Functor for_each_segment(const DoublyConnectedEdgeList& dcel, Functor f)
{
    for (const auto& regionPtr : dcel.regions)
        f = for_each_segment(*regionPtr, f);

    return f;
}

std::pair<bool, std::string> is_valid(const DoublyConnectedEdgeList&);

}

typedef dcel::DoublyConnectedEdgeList DoublyConnectedEdgeList;

}
}
