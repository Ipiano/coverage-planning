#include "quickopp.h"

#include "ads/ccpp/initial-cost/min-across-angles.hpp"
#include "ads/ccpp/optimal-direction/min-across-angles.h"
#include "ads/ccpp/polygon-decomposer/modified-trapezoidal/modified-trapezoidal.h"
#include "ads/ccpp/region-merger/region-merger.h"
#include "ads/ccpp/coordinate-transform.hpp"
#include "ads/ccpp/swath-and-region-producer/swath-and-region-producer.h"
#include "ads/assertion.h"

#include <boost/geometry/algorithms/centroid.hpp>
#include <boost/geometry/algorithms/transform.hpp>
#include <boost/geometry/strategies/transform/matrix_transformers.hpp>
#include <boost/geometry/strategies/transform/inverse_transformer.hpp>

namespace bg = boost::geometry;

namespace ads
{
namespace ccpp
{

std::vector<std::pair<geometry::Ring2d, geometry::MultiLine2d>> quickOpp(const geometry::Polygon2d& polygon, double pathWidth,
                                                                         quantity::Degrees angleTolerance,
                                                                         const interfaces::TurnCostCalculatorIf& turnCalculator)
{
    // Init the cost calculator using the given the turn cost calculator
    ccpp::optimal_direction::MinAcrossAngles dirCalculator(turnCalculator);

    // Determine the initial direction of travel
    ccpp::initial_cost::MinAcrossAngles initialCost(dirCalculator);
    const auto sweepDir = initialCost.calculateInitialDirectionAndCost(polygon).first;

    // Move shape to origin, and rotate so sweep dir is positive X direction
    const auto transform = ccpp::moveToOriginAndRotateCCWTransform(polygon, -sweepDir);

    ccpp::geometry::Polygon2d centeredPolygon;
    boost::geometry::transform(polygon, centeredPolygon, transform);

    // Run the polygon decomposition
    ccpp::polygon_decomposer::ModifiedTrapezoidal decomposer(angleTolerance);
    const auto dcel = decomposer.decomposePolygon(centeredPolygon);

    // Re-combine regions
    ccpp::RegionMerger merger(dirCalculator);
    const auto regionGroups = merger.mergeRegions(dcel).first;

    // Shouldn't ever be possible to produce an invalid DCEL
    ASSERT(dcel.isValid().first);

    // Convert the output from merging regions into rings and groups of swaths
    ccpp::SwathAndRegionProducer swather(pathWidth);
    const auto mergedSwathsAndRegions = swather.produceSwathsAndRegions(dcel, regionGroups);

    // Apply the inverse transform of what was used to center and rotate the shape
    // to put points back where the originated
    const auto invTransform = boost::geometry::strategy::transform::inverse_transformer<double, 2, 2>(transform);

    std::vector<std::pair<geometry::Ring2d, geometry::MultiLine2d>> result(mergedSwathsAndRegions.size());
    auto outputIt = result.begin();
    for (auto inputIt = mergedSwathsAndRegions.begin(); inputIt != mergedSwathsAndRegions.end(); ++inputIt, ++outputIt)
    {
        bg::transform(inputIt->first, outputIt->first, invTransform);
        bg::transform(inputIt->first, outputIt->first, invTransform);
    }

    return result;
}
}
}
