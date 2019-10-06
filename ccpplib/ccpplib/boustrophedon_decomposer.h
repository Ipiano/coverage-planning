#pragma once

#include "polygon_decomposer_if.h"

namespace ccpp {
template<class BoostPolygonType>
class BoustrophedonDecomposer : public PolygonDecomposer<BoostPolygonType>
{
  typedef PolygonDecomposer<BoostPolygonType> super;

public:
  BoustrophedonDecomposer() {}

  std::vector<typename super::RingType> decompose(BoostPolygonType&&)
  {
    return {};
  }
};
}
