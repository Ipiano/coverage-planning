#pragma once

#include "ads/ccpp/interfaces/polygon-decomposer-if.h"
#include "ads/ccpp/typedefs.h"

#include "ads/dcel/dcel.h"

namespace ads
{
namespace ccpp
{
namespace polygon_decomposer
{
namespace modified_trapezoidal
{
class ModifiedTrapezoidal : public interfaces::PolygonDecomposerIf
{
    const ccpp::quantity::Degrees m_angleTolerance;

  public:
    /*!
     * \brief Constructs a decomposer for the modified trapezoidal decomposition
     * \param angleTolerance Max # of degrees between adjacent edges before a region is split
     */
    ModifiedTrapezoidal(const ccpp::quantity::Degrees angleTolerance = ccpp::units::Degree * 361);

    Dcel decomposePolygon(const geometry::Polygon2d& poly) const override;
};
}
using modified_trapezoidal::ModifiedTrapezoidal;
}
}
}
