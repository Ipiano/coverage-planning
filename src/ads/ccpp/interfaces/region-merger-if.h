#pragma once

#include "ads/ccpp/dcel.h"

namespace ads
{
namespace ccpp
{
namespace interfaces
{

class RegionMergerIf
{
  public:
    virtual ~RegionMergerIf() = default;

    /*!
     * \brief Recursively considers pairs of adjacent regions and merges them when appropriate
     *
     * Assumes that the given DCEL follows the rules outlined in the description of
     * PolygonDecomposerIf
     *
     * \param[in, out] dcel Doubly-Connected Edge List with regions to consider merging
     */
    virtual void mergeRegions(DoublyConnectedEdgeList& dcel);
};

}
}
}
