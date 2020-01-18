#pragma once

#include "ads/ccpp/dcel.h"
#include "ads/ccpp/typedefs.h"

namespace ads
{
namespace ccpp
{

void sortEdges(std::vector<const dcel::const_half_edge_t*>& edges, const quantity::Radians sweepDir);
}
}
