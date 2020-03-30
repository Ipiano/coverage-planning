#include "region-merger.h"

#include <unordered_map>

namespace ads
{
namespace ccpp
{
namespace region_merger
{

struct MergeGroupDetails;
struct MergeRegionDetails
{
    // dcel region that could be merged with others
    dcel::region_t* dcelRegion;

    // Optimal direction and cost for the region alone
    quantity::Radians optimalDirection;
    double optimalDirectionCost;

    // Recursive tracker
    bool visited = false;

    // Details about how this region is merged
    // on either side
    std::shared_ptr<MergeGroupDetails> leftMergeGroup  = nullptr;
    std::shared_ptr<MergeGroupDetails> rightMergeGroup = nullptr;
};

struct MergeGroupDetails
{
    MergeRegionDetails* leftRegion;
    MergeRegionDetails* rightRegion;

    quantity::Radians leftRegionDir;
    quantity::Radians rightRegionDir;

    double leftRegionSavings;
    double rightRegionSavings;
};

typedef std::unordered_map<dcel::region_t*, std::unique_ptr<MergeRegionDetails>> MergeRegionDetailsMap;

using interfaces::OptimalDirectionCalculatorIf;
using interfaces::region_merger::MergeRegion;
using interfaces::region_merger::MergeRegionGroup;

std::vector<MergeRegionGroup> mergeRegions(const DoublyConnectedEdgeList& dcel, const OptimalDirectionCalculatorIf& dirCalculator);
void mergeBestAdjacentRegion(dcel::region_t* currentRegion, dcel::region_t* previousRegion,
                             const OptimalDirectionCalculatorIf& dirCalculator, const MergeRegionDetailsMap& mergeDetailsMap);
void mergeTwoRegions(dcel::region_t* left, dcel::region_t* right, dcel::half_edge_t* sharedEdgeLeftSide,
                     const OptimalDirectionCalculatorIf& dirCalculator, const MergeRegionDetailsMap& mergeDetailsMap);

RegionMerger::RegionMerger(const OptimalDirectionCalculatorIf& dirCalculator) : m_dirCalculator(dirCalculator)
{
}

std::vector<MergeRegionGroup> RegionMerger::mergeRegions(const DoublyConnectedEdgeList& dcel)
{
    return region_merger::mergeRegions(dcel, m_dirCalculator);
}

std::vector<MergeRegionGroup> mergeRegions(const DoublyConnectedEdgeList& dcel, const OptimalDirectionCalculatorIf& dirCalculator)
{
    MergeRegionDetailsMap mergeDetails;

    //! 3.4 Determine Optimal Coverage for each Region
    //!
    //! For each region in the list of subregions
    //!     Compute the Optimal Direction of coverage for that region
    //!     Store the result with the region

    for (const auto& regionPtr : dcel.regions)
    {
        const auto optimalDirCost = dirCalculator.calculateOptimalDirectionAndCost(*regionPtr);

        std::unique_ptr<MergeRegionDetails> newRegion(new MergeRegionDetails);
        newRegion->dcelRegion           = regionPtr.get();
        newRegion->optimalDirection     = optimalDirCost.first;
        newRegion->optimalDirectionCost = optimalDirCost.second;

        mergeDetails[regionPtr.get()] = std::move(newRegion);
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
    mergeBestAdjacentRegion(lowerLeftEdge->region, nullptr, dirCalculator, mergeDetails);

    // Build the result
    std::vector<MergeRegionGroup> result;
    while (!mergeDetails.empty())
    {
        // Grab the first region in the list; go left as far as you can
        // and then move right, adding to a result and deleting as you go
        auto regionPtr = mergeDetails.begin()->second.get();
        while (regionPtr->leftMergeGroup)
            regionPtr = regionPtr->leftMergeGroup->leftRegion;

        MergeRegionGroup mergeGroup;
        while (regionPtr != nullptr)
        {
            MergeRegion mr;
            mr.swathDir = regionPtr->leftMergeGroup
                              ? regionPtr->leftMergeGroup->rightRegionDir
                              : (regionPtr->rightMergeGroup ? regionPtr->rightMergeGroup->leftRegionDir : regionPtr->optimalDirection);
            mr.dcelRegion = regionPtr->dcelRegion;

            mergeGroup.regionsToMerge.push_back(mr);

            // Move to next region and delete the current one
            MergeRegionDetails* nextRegion = nullptr;
            if (regionPtr->rightMergeGroup)
                nextRegion = regionPtr->rightMergeGroup->rightRegion;
            mergeDetails.erase(regionPtr->dcelRegion);
            regionPtr = nextRegion;
        }

        result.push_back(mergeGroup);
    }

    return result;
}

static bool pointsHaveSameXCoord(const dcel::vertex_t* v1, const dcel::vertex_t* v2)
{
    return std::abs(v1->location.x() - v2->location.x()) < 0.00001;
}

static quantity::Radians normal(const quantity::Radians angle)
{
    const static auto quarter = static_cast<quantity::Radians>(units::Degree * 90);

    // Keep result in [0, PI)
    return angle >= quarter ? angle - quarter : angle + quarter;
}

static bool same(const quantity::Radians left, quantity::Radians right)
{
    return std::abs((left - right).value()) < 0.000001;
}

// Recursively processes all regions to the right of nextRegion,
// then evaluates merging nextRegion with each of the regions it is connected to on the right
//
// Change from original algorithm: nextRegion may be connected to > 4 regions; so all will be processed
void mergeBestAdjacentRegion(dcel::region_t* const currentRegion, dcel::region_t* const previousRegion,
                             const OptimalDirectionCalculatorIf& dirCalculator, const MergeRegionDetailsMap& mergeDetailsMap)
{
    assert(currentRegion != nullptr);
    assert(mergeDetailsMap.count(currentRegion) > 0);

    const auto& mergeDetails = mergeDetailsMap.find(currentRegion)->second;
    std::vector<dcel::half_edge_t*> leftSideAdjacentEdges;

    //! 1. Set currentEdge equal to any edge on the subregion to be processed.
    //!     The current edge represents the common edge between two faces
    dcel::half_edge_t* currentEdge = currentRegion->edge;

    //! 2. Mark currentRegion as visited
    mergeDetails->visited = true;

    //! 3. Set startEdge equal to the current edge
    const auto startEdge = currentEdge;

    //! 4. While currentEdge != startEdge
    do
    {
        //! a. Set secondRegion equal to the face of currentEdge's twin
        // Also double check for degenerate case of 0-length edge
        // Also check if the adjacent region is not the one we came from (see comment in c below about why we can do this)
        if (currentEdge->twin != nullptr && currentEdge->origin != currentEdge->next->origin && currentEdge->twin->region != previousRegion)
        {
            auto secondRegion = currentEdge->twin->region;
            if (!pointsHaveSameXCoord(currentEdge->origin, currentEdge->next->origin))
                throw std::invalid_argument("invalid dcel: regions share non-vertical edge");

            // Since regions are only adjacent to other regions along contiguous vertical edges,
            // we can determine which side the adjacent region is on by the direction of the shared edge
            const bool rightSideAdjacent = currentEdge->origin->location.y() > currentEdge->next->origin->location.y();

            //! b. if secondRegion != previousRegion and it adjoins on the right [of currentRegion]
            if (rightSideAdjacent)
            {
                //! i. Recursively call mergeBestAdjacentRegion(secondRegion, currentRegion)
                mergeBestAdjacentRegion(secondRegion, currentRegion, dirCalculator, mergeDetailsMap);

                //! ii. Call mergeTwoRegions(currentRegion, secondRegion, currentEdge)
                mergeTwoRegions(currentRegion, secondRegion, currentEdge, dirCalculator, mergeDetailsMap);
            }

            //! c. Else if secondRegion == previousRegion and it ajoins on the left [of current region]
            //!     add it to the list of left edges to process later
            // This seems like a typo; I'm pretty sure it should be if it's != previousRegion. Otherwise
            // there's a bunch of regions that never get processed, and when we go to proceess the left
            // edges at the end, we'll just ignore them all becuase that region is the one we came from;
            // it's already been processed
            else
            {
                leftSideAdjacentEdges.push_back(currentEdge);
            }
        }
        currentEdge = currentEdge->next;
    } while (currentEdge != startEdge);

    //! 5. For each edge, leftEdge, in the list of left edges to process later
    for (auto* leftEdge : leftSideAdjacentEdges)
    {
        //! a. If the region on the opposite side of the edge is not null and it has not been visited
        assert(leftEdge->twin != nullptr);

        auto leftRegion = leftEdge->twin->region;
        assert(mergeDetailsMap.count(leftRegion) > 0);

        const auto& leftMergeDetails = mergeDetailsMap.find(leftRegion)->second;
        if (!leftMergeDetails->visited)
        {
            //! i. Recursively call mergeBestAdjacentRegion(the other side of leftEdge, currentRegion)
            mergeBestAdjacentRegion(leftRegion, currentRegion, dirCalculator, mergeDetailsMap);

            //! ii. Call mergeTwoRegions(currentRegion, leftEdge)
            mergeTwoRegions(leftRegion, currentRegion, leftEdge->twin, dirCalculator, mergeDetailsMap);
        }
    }

    //! 6. Return currentRegion
}

void mergeTwoRegions(dcel::region_t* left, dcel::region_t* right, dcel::half_edge_t* sharedEdgeLeftSide,
                     const OptimalDirectionCalculatorIf& dirCalculator, const MergeRegionDetailsMap& mergeDetailsMap)
{
    assert(left != nullptr);
    assert(mergeDetailsMap.count(left) > 0);
    assert(right != nullptr);
    assert(mergeDetailsMap.count(right) > 0);

    const auto& leftRegionMergeDetails  = mergeDetailsMap.find(left)->second;
    const auto& rightRegionMergeDetails = mergeDetailsMap.find(right)->second;

    //! 1. Set previousMergedFace = the merged region assigned to the left region
    // This isn't used for anything later in the algorithm... so why do this?

    // Did we decide it's advantageous to merge them?
    bool mergeFound = false;

    // Details about how we're going to merge them (if we do)
    std::shared_ptr<MergeGroupDetails> newMergeGroup(new MergeGroupDetails);
    newMergeGroup->leftRegion  = leftRegionMergeDetails.get();
    newMergeGroup->rightRegion = rightRegionMergeDetails.get();

    // Get the vertices on the shared edge
    dcel::half_edge_t* sharedEdgeLeftSideTop = sharedEdgeLeftSide;
    while (sharedEdgeLeftSideTop->prev->twin == sharedEdgeLeftSideTop->twin->next)
        sharedEdgeLeftSideTop = sharedEdgeLeftSideTop->prev;

    dcel::half_edge_t* sharedEdgeLeftSideBottom = sharedEdgeLeftSide;
    while (sharedEdgeLeftSideBottom->next->twin == sharedEdgeLeftSideBottom->twin->prev)
        sharedEdgeLeftSideBottom = sharedEdgeLeftSideBottom->next;

    const auto topSharedVertex    = sharedEdgeLeftSideTop->origin->location;
    const auto bottomSharedVertex = sharedEdgeLeftSideBottom->twin->origin->location;

    const static auto verticalAngle = static_cast<quantity::Radians>(units::Degree * 90);

    // Calculate cost of not merging the regions and
    // just using their optimal directions separately.
    // If we can beat this with a merge strategy, then we go
    // with it
    const double optimalUnmergedCost = leftRegionMergeDetails->optimalDirectionCost + rightRegionMergeDetails->optimalDirectionCost;

    //! 2. If the two regions have the same optimal direction, merge them immediately
    if (same(leftRegionMergeDetails->optimalDirection, rightRegionMergeDetails->optimalDirection))
    {
        mergeFound           = true;
        const auto normalDir = normal(leftRegionMergeDetails->optimalDirection);

        //! a. Calculate the cost savings if both regions are covered in the direction normal
        //!     to their current optimal direction.
        const auto normalLeftCost     = dirCalculator.totalCost(*left, normalDir);
        const auto normalRightCost    = dirCalculator.totalCost(*right, normalDir);
        const auto normalUnmergedCost = normalLeftCost + normalRightCost;

        const double normalSavings    = dirCalculator.edgeCost(topSharedVertex, bottomSharedVertex, normalDir);
        const double normalMergedCost = normalUnmergedCost - normalSavings * 2;

        const double optimalSavings = dirCalculator.edgeCost(topSharedVertex, bottomSharedVertex, leftRegionMergeDetails->optimalDirection);
        const double optimalMergedCost = optimalUnmergedCost - optimalSavings * 2;

        //! b. If the cost savings is > 0, merge the regions using the perpendicular direction of travel.
        //!     Store the cost savings with the merged region.
        if (normalMergedCost < optimalMergedCost)
        {
            newMergeGroup->leftRegionDir  = normalDir;
            newMergeGroup->rightRegionDir = normalDir;

            newMergeGroup->leftRegionSavings  = leftRegionMergeDetails->optimalDirectionCost - (normalLeftCost - normalSavings);
            newMergeGroup->rightRegionSavings = rightRegionMergeDetails->optimalDirectionCost - (normalRightCost - normalSavings);
        }
        //! c. Otherwise merge the regions with the optimal direction. Store the merged region with the
        //!     cost savings from omitting the shared edge
        else
        {
            newMergeGroup->leftRegionDir  = leftRegionMergeDetails->optimalDirection;
            newMergeGroup->rightRegionDir = leftRegionMergeDetails->optimalDirection;

            // We get the same savings on both sides of the line
            // and since it's the optimal direction it's already relative to
            // the baseline cost value
            newMergeGroup->leftRegionSavings  = optimalSavings;
            newMergeGroup->rightRegionSavings = optimalSavings;
        }
    }
    //! 3. Otherwise If the optimal paths covering the two regions intersect the shared edge
    else if (!same(leftRegionMergeDetails->optimalDirection, verticalAngle) &&
             !same(rightRegionMergeDetails->optimalDirection, verticalAngle))
    {
        //! a. Calculate the savings from merging the two regions.
        const double leftSavings  = dirCalculator.edgeCost(topSharedVertex, bottomSharedVertex, leftRegionMergeDetails->optimalDirection);
        const double rightSavings = dirCalculator.edgeCost(topSharedVertex, bottomSharedVertex, rightRegionMergeDetails->optimalDirection);

        //! b. Merge them if the savings is > 0
        //... why wouldn't it be? Maybe if you take into account
        // the current merge strategy for the left and right instead
        // of their unmerged costs? But we do that at the end below
        if (leftSavings + rightSavings > 0)
        {
            mergeFound = true;

            newMergeGroup->leftRegionDir  = leftRegionMergeDetails->optimalDirection;
            newMergeGroup->rightRegionDir = rightRegionMergeDetails->optimalDirection;

            newMergeGroup->leftRegionSavings  = leftSavings;
            newMergeGroup->rightRegionSavings = rightSavings;
        }
    }
    //! 3(b). Otherwise
    else
    {
        const std::array<quantity::Radians, 4> dirsToCheck = {
            leftRegionMergeDetails->optimalDirection, rightRegionMergeDetails->optimalDirection,
            normal(leftRegionMergeDetails->optimalDirection), normal(rightRegionMergeDetails->optimalDirection)};

        //! a. Calculate cost savings using optimal direction from the left region
        //! b. Calculate cost savings using optimal direction from the right region
        //! c. Calculate cost savings using normal to optimal direction from left region
        //! d. Calculate cost savings using normal to optimal direction from right region
        //! e. If largest cost savings > 0, merge with the corresponding pattern and store the merged region
        //!     and savings

        double bestCost = optimalUnmergedCost;
        for (const auto& dir : dirsToCheck)
        {
            const double leftCost = same(dir, leftRegionMergeDetails->optimalDirection) ? leftRegionMergeDetails->optimalDirectionCost
                                                                                        : dirCalculator.totalCost(*left, dir);

            const double rightCost = same(dir, rightRegionMergeDetails->optimalDirection) ? rightRegionMergeDetails->optimalDirectionCost
                                                                                          : dirCalculator.totalCost(*right, dir);

            const double unmergedCost = leftCost + rightCost;
            const double savings      = dirCalculator.edgeCost(topSharedVertex, bottomSharedVertex, dir);
            const double mergedCost   = unmergedCost - savings * 2;

            if (mergedCost < bestCost)
            {
                bestCost   = mergedCost;
                mergeFound = true;

                newMergeGroup->leftRegionDir  = dir;
                newMergeGroup->rightRegionDir = dir;

                newMergeGroup->leftRegionSavings  = leftRegionMergeDetails->optimalDirectionCost - (leftCost - savings);
                newMergeGroup->rightRegionSavings = rightRegionMergeDetails->optimalDirectionCost - (rightCost - savings);
            }
        }
    }

    if (mergeFound)
    {

        //! 4. If the regions were merged and the left region was previously associated with a different region
        //!     a. Compare the savings for the new merge pattern with the savings for the previous merge pattern
        //!     b. If it is more cost effective to merge with the new pattern, remove the left region from the region
        //!         it was previously merged with
        //! 5. If the regions were merged and the right region was previously associated with another merged region
        //!     a. Compare the savings for using the new merge strategy and the previous one
        //!     b. If it is more cost effective to merge with the new merge strategy than the previous one, remove
        //!         the second region from the merged region it was previously associated with and set its region to
        //!         the new merge region
        //
        // These steps are modified slightly. If we merge these together, there's some amount of previous savings
        // that gets lost; so we weigh the savings from merging them against all the savings lost. Since both
        // regions may have up to two merges already, there's potentially four merges we have to break to merge
        // these; so it has to be REALLY worth it in that case
        //
        // There is a little bit of a silver lining though; if the outside merges already existing (left side
        // of left region, right side of right region) merged using the same direction as the merge we're considering
        // for these two, then those merges don't have to be broken, we can just chain them together.

        double savingsLost = 0;

        // If the right region previously had a left merge; this would replace it; and if the right region previously
        // had a right merge, it can only persist if we didn't change the direction of the right region
        bool loseRightMerge = false;
        if (rightRegionMergeDetails->leftMergeGroup)
        {
            savingsLost += rightRegionMergeDetails->leftMergeGroup->rightRegionSavings;
            savingsLost += rightRegionMergeDetails->leftMergeGroup->leftRegionSavings;
        }

        if (rightRegionMergeDetails->rightMergeGroup &&
            !same(newMergeGroup->rightRegionDir, rightRegionMergeDetails->rightMergeGroup->leftRegionDir))
        {
            loseRightMerge = true;
            savingsLost += rightRegionMergeDetails->rightMergeGroup->leftRegionSavings;
            savingsLost += rightRegionMergeDetails->rightMergeGroup->rightRegionSavings;
        }

        // If the left region previously had a right merge; this would replace it; and if the left region previously
        // had a left merge, it can only persist if we didn't change the direction of the left region
        bool loseLeftMerge = false;
        if (leftRegionMergeDetails->rightMergeGroup)
        {
            savingsLost += leftRegionMergeDetails->rightMergeGroup->rightRegionSavings;
            savingsLost += leftRegionMergeDetails->rightMergeGroup->leftRegionSavings;
        }

        if (leftRegionMergeDetails->leftMergeGroup &&
            !same(newMergeGroup->leftRegionDir, leftRegionMergeDetails->leftMergeGroup->rightRegionDir))
        {
            loseLeftMerge = true;
            savingsLost += leftRegionMergeDetails->leftMergeGroup->leftRegionSavings;
            savingsLost += leftRegionMergeDetails->leftMergeGroup->rightRegionSavings;
        }

        if (savingsLost < newMergeGroup->rightRegionSavings + newMergeGroup->leftRegionSavings)
        {
            // Reassign the right region to merge with this one on the left
            // and unassign the previous region it was merged with
            if (rightRegionMergeDetails->leftMergeGroup)
                rightRegionMergeDetails->leftMergeGroup->leftRegion->rightMergeGroup.reset();
            rightRegionMergeDetails->leftMergeGroup = newMergeGroup;

            // Reassign the left region to merge with this one on the right
            // and unassign the previous region it was merged with
            if (leftRegionMergeDetails->rightMergeGroup)
                leftRegionMergeDetails->rightMergeGroup->rightRegion->leftMergeGroup.reset();
            leftRegionMergeDetails->rightMergeGroup = newMergeGroup;

            // If the angle changes on right side, unassign the merge on the right side
            // of the right region
            if (loseRightMerge)
            {
                rightRegionMergeDetails->rightMergeGroup->rightRegion->leftMergeGroup.reset();
                rightRegionMergeDetails->rightMergeGroup.reset();
            }

            // If the angle changes on left side, unassign the merge on the left side
            // of the left region
            if (loseLeftMerge)
            {
                leftRegionMergeDetails->leftMergeGroup->leftRegion->rightMergeGroup.reset();
                leftRegionMergeDetails->leftMergeGroup.reset();
            }
        }
    }
}

}
}
}
