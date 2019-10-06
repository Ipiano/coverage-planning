#pragma once

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/ring.hpp>

#include <vector>

namespace ccpp {

template<class BoostPolygonType>
class PolygonDecomposer
{
protected:
  typedef
    typename boost::geometry::point_type<BoostPolygonType>::type PointType;
  typedef boost::geometry::model::ring<PointType> RingType;

public:
  PolygonDecomposer() {}

  virtual std::vector<RingType> decompose(BoostPolygonType&&) = 0;
};
}
