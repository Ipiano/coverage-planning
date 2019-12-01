#pragma once

#include "ads/ccpp/optimal-direction/optimal-direction-concept.h"

namespace ads {
namespace ccpp {
namespace initial_cost {

template<class InitialCostImpl>
using InitialCostConcept =  optimal_direction::OptimalDirectionConcept<InitialCostImpl>;

}
}
}
