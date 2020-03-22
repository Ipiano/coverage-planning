#include "region-merger.h"

namespace ads
{
namespace ccpp
{
namespace region_merger
{

void mergeRegions(DoublyConnectedEdgeList& dcel, const interfaces::OptimalDirectionCalculatorIf& dirCalculator);
void mergeBestAdjacentRegion(dcel::region_t* currentRegion, dcel::region_t* previousRegion,
                             const interfaces::OptimalDirectionCalculatorIf& dirCalculator);
void mergeTwoRegions(dcel::region_t* left, dcel::region_t* right, dcel::half_edge_t* leftSideAdjacentEdge);

RegionMerger::RegionMerger(const interfaces::OptimalDirectionCalculatorIf& dirCalculator) : m_dirCalculator(dirCalculator)
{
}

void RegionMerger::mergeRegions(DoublyConnectedEdgeList& dcel)
{
    region_merger::mergeRegions(dcel, m_dirCalculator);
}

void mergeRegions(DoublyConnectedEdgeList& dcel, const interfaces::OptimalDirectionCalculatorIf& dirCalculator)
{
    if (dcel.regions.size() < 2)
        return;

    //! 3.4 Determine Optimal Coverage for each Region
    //!
    //! For each region in the list of subregions
    //!     Compute the Optimal Direction of coverage for that region
    //!     Store the result with the region

    for (const auto& regionPtr : dcel.regions)
    {
        const auto optimalDir = dirCalculator.calculateOptimalDirection(*regionPtr);
        // TODO: Save this
    }

    //! 3.5 Merge the Regions
    //!
    //! Set firstFace equal to the left-most face in the DCEL
    //! Call MergeBestChild(firstFace, nullptr)
    //! Return modified DCEL

    // Determine the left-most face on the dcel
    dcel::half_edge_t* lowerLeftEdge = dcel.edges[0].get();
    for (const auto& halfEdgePtr : dcel.edges)
    {
        if (halfEdgePtr->origin->location.x() < lowerLeftEdge->origin->location.x())
            lowerLeftEdge = halfEdgePtr.get();
    }
    mergeBestAdjacentRegion(lowerLeftEdge->region, nullptr, dirCalculator);
}

// Recursively processes all regions to the right of nextRegion,
// then evaluates merging nextRegion with each of the regions it is connected to on the right
//
// Change from original algorithm: nextRegion may be connected to > 4 regions; so all will be processed
void mergeBestAdjacentRegion(dcel::region_t* const currentRegion, dcel::region_t* const previousRegion,
                             const interfaces::OptimalDirectionCalculatorIf& dirCalculator)
{
    //! 1. Set currentEdge equal to any edge on the subregion to be processed.
    //!     The current edge represents the common edge between two faces
    //! 2. Mark currentRegion as visited
    //! 3. Set startEdge equal to the current edge
    //! 4. While currentEdge != startEdge
    //!     a. Set secondRegion equal to the face of currentEdge's twin
    //!     b. if secondRegion != previousRegion and it adjoins on the right [of currentRegion]
    //!         i. Recursively call mergeBestAdjacentRegion(secondRegion, currentRegion)
    //!         ii. Call mergeTwoRegions(currentRegion, secondRegion, currentEdge)
    //!     c. Else if secondRegion == prevoiusRegion and it ajoins on the left [of current region]
    //!         add it to the list of left edges to process later
    //! 5. For each edge, leftEdge, in the list of left edges to process later
    //!     a. If the region on the opposite side of the edge is not null and it has not been visited
    //!         i. Recursively call mergeBestAdjacentRegion(the other side of leftEdge, currentRegion)
    //!         ii. Call mergeTwoRegions(currentRegion, leftEdge)
    //! 6. Return currentRegion
}

void mergeTwoRegions(dcel::region_t* left, dcel::region_t* right, dcel::half_edge_t* leftSideAdjacentEdge)
{
    //! 1. Set previousMergedFace = the merged region assigned to the left region
    //! 2. If the two regions have the same optimal direction, merge them immediately
    //!     a. Calculate the cost savings if both regions are covered in the direction normal
    //!         to their current optimal direction.
    //!     b. If the cost savings is > 0, merge the regions using the perpendicular direction of travel.
    //!         Store the cost savings with the merged region.
    //!     c. Otherwise merge the regions with the optimal direction. Store the merged region with the
    //!         cost savings from omitting the shared edge
    //! 3. Otherwise If the optimal paths covering the shared region intersect the shared edge
    //!     a. Calculate the savings from merging the two regions.
    //!     b. Merge them if the savings is > 0
    //! 3(b). Otherwise
    //!     a. Calculate cost savings using optimal direction from the left region
    //!     b. Calculate cost savings using optimal direction from the right region
    //!     c. Calculate cost savings using normal to optimal direction from left region
    //!     d. Calculate cost savings using normal to optimal direction from right region
    //!     e. If largest cost savings > 0, merge with the corresponding pattern and store the merged region
    //!         and savings
    //! 4. If the regions were merged and the left region was previously associated with a different region
    //!     a. Compare the savings for the new merge pattern with the savings for the previous merge pattern
    //!     b. If it is more cost effective to merge with the new pattern, remove the left region from the region
    //!         it was previously merged with
    //! 5. If the regions were merged and the right region was previously associated with another merged region
    //!     a. Compare the savings for using the new merge strategy and the previous one
    //!     b. If it is more cost effective to merge with the new merge strategy than the previous one, remove
    //!         the second region from the merged region it was previously associated with and set its region to
    //!         the new merge region
}

}
}
}
