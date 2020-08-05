#pragma once

#include "ads/ccpp/typedefs.h"
#include "ads/ccpp/interfaces/turn-cost-calculator-if.h"

namespace ads
{
namespace ccpp
{

std::vector<std::pair<geometry::Ring2d, geometry::MultiLine2d>> quickOpp(const geometry::Polygon2d& polygon, double pathWidth,
                                                                         const interfaces::TurnCostCalculatorIf& turnCalculator);

} // namespace ccpp
} // namespace ads
