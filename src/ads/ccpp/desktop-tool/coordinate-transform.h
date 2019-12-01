#pragma once

#include "ads/ccpp/desktop-tool/typedefs.h"

namespace ads
{
namespace ccpp
{
namespace desktop_tool
{

template<class PointT1, class PointT2>
PointT1 cast_point(const PointT2& p1)
{
    static_assert(boost::geometry::dimension<PointT1>::value == 2, "Cannot cast non-2d geometry");
    static_assert(boost::geometry::dimension<PointT2>::value == 2, "Cannot cast non-2d geometry");

    return boost::geometry::make<PointT1>(boost::geometry::get<0>(p1), boost::geometry::get<1>(p1));
}

template<class RingT1, class RingT2>
RingT1 cast_ring(const RingT2& r1)
{
    RingT1 r2;
    r2.resize(r1.size());

    typedef typename boost::geometry::point_type<RingT1>::type PointT1;
    typedef typename boost::geometry::point_type<RingT2>::type PointT2;

    std::transform(r1.begin(), r1.end(), r2.begin(), &cast_point<PointT1, PointT2>);

    return r2;
}

template<class PolygonT1, class PolygonT2>
PolygonT1 cast_polygon(const PolygonT2& p1)
{
    PolygonT1 p2;

    typedef typename boost::geometry::ring_type<PolygonT1>::type RingT1;
    typedef typename boost::geometry::ring_type<PolygonT2>::type RingT2;

    p2.outer() = cast_ring<RingT1, RingT2>(p1.outer());
    p2.inners().resize(p1.inners().size());

    std::transform(p1.inners().begin(), p1.inners().end(), p2.inners().begin(), &cast_ring<RingT1, RingT2>);

    return p2;
}

}
}
}
