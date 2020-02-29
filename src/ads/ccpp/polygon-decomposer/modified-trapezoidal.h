#pragma once

#include "ads/ccpp/interfaces/polygon-decomposer-if.h"
#include "ads/ccpp/dcel.h"
#include "ads/ccpp/typedefs.h"

namespace ads
{
namespace ccpp
{
namespace polygon_decomposer
{

class ModifiedTrapezoidal : public interfaces::PolygonDecomposerIf
{
  public:
    ModifiedTrapezoidal();

    DoublyConnectedEdgeList decomposePolygon(const geometry::Polygon2d& poly) const override;
};
}
}
}
