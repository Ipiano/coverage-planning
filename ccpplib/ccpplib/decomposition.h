#pragma once

#include <boost/geometry.hpp>
#include <boost/geometry/core/point_type.hpp>
#include <boost/geometry/geometries/geometries.hpp>

namespace ccpp {

template<class BoostPolygonType>
class PolygonDecomposer
{
    //typedef typename boost::geometry::traits::point_type<BoostPolygonType>::type PointType;
    //typedef boost::geometry::model::ring<PointType> RingType;
public:
    PolygonDecomposer(){}

    //virtual void std::vector<RingType> decompose(BoostPolygonType&&) = 0;
};

}
