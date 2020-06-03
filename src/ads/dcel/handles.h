#pragma once

#include <boost/serialization/strong_typedef.hpp>

namespace ads
{
namespace dcel
{
//! Strong types for the handles used to reference pieces of a Dcel
BOOST_STRONG_TYPEDEF(unsigned long long, HalfEdgeHandle);
BOOST_STRONG_TYPEDEF(unsigned long long, VertexHandle);
BOOST_STRONG_TYPEDEF(unsigned long long, RegionHandle);

//! Defined 'unused' or default values for the handles
const static VertexHandle NoVertex     = VertexHandle(0);
const static HalfEdgeHandle NoHalfEdge = HalfEdgeHandle(0);
const static RegionHandle NoRegion     = RegionHandle(0);

namespace hash
{
//! Hash struct for handles so they can be used as map keys
template <class HandleT> struct Handle
{
    std::size_t operator()(HandleT h) const { return std::hash<unsigned long long>()(h.t); }
};
}
}
}
