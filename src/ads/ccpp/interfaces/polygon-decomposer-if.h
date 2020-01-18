#pragma once

#include "ads/ccpp/dcel.h"

#include <vector>

namespace ads {
namespace ccpp {
namespace interfaces {

class PolygonDecomposerIf
{
public:
    virtual ~PolygonDecomposerIf() = default;

    /*!
     * \brief decomposePolygon
     * \param dcel
     */
    virtual void decomposePolygon(DoublyConnectedEdgeList& dcel) const = 0;
};
}
}
}
