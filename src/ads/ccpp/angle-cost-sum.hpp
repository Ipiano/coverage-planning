#pragma once

#include "ads/ccpp/typedefs.h"
#include "ads/ccpp/interfaces/turn-cost-calculator-if.h"

namespace ads
{
namespace ccpp
{

struct AngleCostSum
{
    quantity::Radians m_angle;
    double m_totalCost;
    const interfaces::TurnCostCalculatorIf& m_calculator;

  public:
    AngleCostSum(const interfaces::TurnCostCalculatorIf& calculator, const quantity::Radians angle)
        : m_angle(angle), m_totalCost(0), m_calculator(calculator)
    {
    }

    template <class SegmentT> void operator()(const SegmentT& segment) { m_totalCost += m_calculator.calculateTurnCost(segment, m_angle); }

    double cost() const { return m_totalCost; }
};
}
}
