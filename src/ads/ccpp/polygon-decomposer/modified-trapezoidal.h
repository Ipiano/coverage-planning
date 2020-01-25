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
    ModifiedTrapezoidal(const quantity::Radians sweepDir);

    DoublyConnectedEdgeList decomposePolygon(const geometry::Polygon2d& poly) const override;

  private:
    quantity::Radians m_sweepDir;
};
}
}
}
