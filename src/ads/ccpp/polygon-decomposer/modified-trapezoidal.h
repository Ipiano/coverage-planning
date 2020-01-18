#pragma once

#include "ads/ccpp/interfaces/polygon-decomposer-if.h"
#include "ads/ccpp/dcel.h"
#include "ads/ccpp/typedefs.h"

namespace ads {
namespace ccpp {
namespace polygon_decomposer {

class ModifiedTrapezoidal : public interfaces::PolygonDecomposerIf
{
public:
    ModifiedTrapezoidal(const quantity::Radians sweepDir);

    void decomposePolygon(DoublyConnectedEdgeList& dcel) const override;

private:
    quantity::Radians m_sweepDir;

    void decompose(std::vector<const dcel::const_half_edge_t *> &sortedEdges, DoublyConnectedEdgeList& dcel) const;
};

}
}
}
