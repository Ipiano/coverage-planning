#pragma once

#include "ads/ccpp/typedefs.h"
#include "ads/ccpp/interfaces/turn-cost-calculator-if.h"

namespace ads
{
namespace ccpp
{

/*!
 * \brief Executes the QuickOpp algorithm implementation in this library
 *
 * The QuickOpp algorithm, as described in
 * "Complete coverage path planning in an agricultural environment." (Driscoll, Theresa Marie. 2011).
 * is an algorithm for producing solutions to the Complete Coverage Path Planning (CCPP) problem.
 * It is designed specifically for use in agricultural applications and includes heuristics for
 * such applications.
 *
 * This algorithm consists of four main steps
 * * Determine initial cost and direction
 *      This step will check all angles in a range to determine which would produce
 *      the lowest cost if the entire field were covered using this direction.
 *
 * * Decompose the polygon
 *      Decomposition is done using a boustrephodon decomposition followed by futher
 *      splitting of the regions produced. The secondary splitting is done based on
 *      the relative angles of adjacent edges in each region. If adjacent edges form
 *      too sharp of an angle (as determined by the angleTolerance input), the region
 *      will be split.
 *
 * * Merge adjacent regions
 *      After the polygon is decomposed into regions, some of those regions are re-combined.
 *      Adjacent regions will be combined if the overall cost of covering them together is lower
 *      than the cost of covering them separately. This overall cost may use a different direction
 *      of travel than would be the best case for either region separately.
 *
 * * Produce final output
 *      Groups of regions to be merged together are processed to produce a final result consisiting
 *      of a list of rings and the paths to be traveled within them. These paths are spaced out
 *      according to the input pathWidth.
 *
 * While the input shape is not expected to adhere to any specific units, a number of internal constants
 * have been chosen with the agricultural applications in mind. For best results, input shapes should be
 * on a 1-meter coordinate grid and contain at least a square meter of area.
 *
 * It may be desirable to tweak the values passed to the internal pieces of this algorithm or to
 * outright replace them, depending on your use-case. In this case, this function can serve as
 * an example of what operations need to be performed and in what order to execute this algorithm
 * using the pieces in this library.
 *
 * \todo Remove or expose internal constants that affect units of input data
 *
 * \param polygon Shape to run QuickOpp on. This must be a valid boost polygon. This means that
 *                it must boost::geometry::is_valid must evaluate to true when passed this shape. The
 *                shape should have exactly one outer ring and any number of internal rings representing
 *                holes in the area.
 * \param pathWidth Width of the vehicle to produce output paths for
 * \param angleTolerance Tolerance (in degrees) for the secondary decomposition step. After the initial
 *                decomposition, regions will be split if adjacent edges have a relative angle greater than
 *                this value.
 * \param turnCalculator Cost function. This should be some object implementing the TurnCostCalculator interface
 *                in ads/ccpp/interfaces/TurnCostCalculatorIf.h A basic implementation which evaluates the cost
 *                of a standard U-Shaped turn around has been included in ads/ccpp/turn-cost/u-shaped.h
 *
 * \return A list of pairs, where each pair contains an outer ring for an area of the shape and the set of paths
 *         that should be driven on to cover that area.
 *
 * \throws std::invalid_argument if the input geometry is an invalid shape
 * \throws ads::AssertionFailure if an internal assertion fails due to a bug in the algorithm
 */
std::vector<std::pair<geometry::Ring2d, geometry::MultiLine2d>> quickOpp(const geometry::Polygon2d& polygon, double pathWidth,
                                                                         ccpp::quantity::Degrees angleTolerance,
                                                                         const interfaces::TurnCostCalculatorIf& turnCalculator);

} // namespace ccpp
} // namespace ads
