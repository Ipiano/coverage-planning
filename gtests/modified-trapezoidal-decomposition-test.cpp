#include "ads/ccpp/polygon-decomposer/modified-trapezoidal/modified-trapezoidal.h"

#include <boost/geometry/algorithms/distance.hpp>
#include <boost/geometry/multi/geometries/multi_point.hpp>
#include <boost/geometry/algorithms/intersection.hpp>

#include <gtest/gtest.h>

#include <iterator>
#include <sstream>
#include <unordered_set>
#include <iostream>

using namespace testing;
using namespace ads;
using namespace ccpp;
namespace bg = boost::geometry;

// Utility functions
std::string to_string(const geometry::Point2d& pt)
{
    std::stringstream ss;
    ss << std::fixed << std::setprecision(2) << "<" << pt.x() << "," << pt.y() << ">";
    return ss.str();
}

bool same(const geometry::Point2d& pt1, const geometry::Point2d& pt2)
{
    return bg::distance(pt1, pt2) < 0.0001;
}

struct DcelDescription
{
    std::vector<std::vector<geometry::Point2d>> regions;
};

// GTest checker that a DCEL object is valid. Meaning it does not
// reference pointers outside itself, and it does not do anything
// logically incorrect (e.g., adjacent edges don't have same region)
AssertionResult DcelIsValid(const Dcel& dcel)
{
    bool isValid;
    std::string errMsg;

    std::tie(isValid, errMsg) = dcel.isValid();

    return isValid ? AssertionSuccess() : AssertionFailure() << errMsg;
}

// GTest checker that a DCEL has a specific structure, meaning
// that it has the expected regions and that each region has the expected
// set of points in the correct order
AssertionResult DcelHasRegion(const Dcel& dcel, const std::vector<geometry::Point2d>& expectedRegion)
{
    if (expectedRegion.empty())
        return AssertionFailure() << "dcel description contains empty region";

    int bestCase           = 0;
    AssertionResult result = AssertionFailure();

    for (const auto& resultRegion : dcel.regions())
    {
        bool foundStart     = false;
        dcel::HalfEdge edge = resultRegion.edge();

        do
        {
            if (same(edge.origin().point(), expectedRegion.front()))
            {
                foundStart = true;
                break;
            }
        } while ((edge = edge.next()) != resultRegion.edge());

        if (!foundStart)
        {
            if (bestCase < 1)
            {
                bestCase = 1;
                result   = AssertionFailure() << "expected region start point" << to_string(expectedRegion.front()) << "not found";
            }
            continue;
        }

        const dcel::HalfEdge firstEdge = edge;
        unsigned int matched           = 0;
        auto expectedPointIt           = expectedRegion.begin();

        bool matchedAllPoints = true;
        do
        {
            if (!same(edge.origin().point(), *expectedPointIt))
            {
                if (bestCase < 2)
                {
                    bestCase = 2;
                    result   = AssertionFailure()
                             << "point " << std::distance(expectedRegion.begin(), expectedPointIt) << " does not match :: "
                             << "got " << to_string(edge.origin().point()) << " expected " << to_string(*expectedPointIt);
                }
                matchedAllPoints = false;
                break;
            }
            matched++;
        } while ((edge = edge.next()) != firstEdge && ++expectedPointIt != expectedRegion.end());

        if (!matchedAllPoints)
            continue;

        if (matched != expectedRegion.size())
        {
            if (bestCase < 3)
            {
                bestCase = 3;
                result = AssertionFailure() << "does not contain enough points :: got " << matched << " expected " << expectedRegion.size();
            }
            continue;
        }

        if (edge != firstEdge)
        {
            do
            {
                matched++;
            } while ((edge = edge.next()) != firstEdge);

            if (bestCase < 3)
            {
                result = AssertionFailure() << "contains too many points :: got " << matched << " expected " << expectedRegion.size();
            }
            continue;
        }

        return AssertionSuccess();
    }

    return result;
}

AssertionResult DcelMatchesDescription(const Dcel& dcel, const DcelDescription& description)
{
    const auto dcelRegions = dcel.regions().size();
    if (dcelRegions != description.regions.size())
        return AssertionFailure() << "dcel contains wrong number of regions :: got" << dcelRegions << " expected "
                                  << description.regions.size();

    for (size_t i = 0; i < description.regions.size(); i++)
    {
        auto resultThisRegion = DcelHasRegion(dcel, description.regions[i]);
        if (!resultThisRegion)
            return resultThisRegion << " (region " << i << ")";
    }

    return AssertionSuccess();
}

typedef std::pair<geometry::Polygon2d, DcelDescription> PolygonAndDcel;

class ModifiedTrapezoidalPolygonDecomposition : public testing::TestWithParam<PolygonAndDcel>
{
  protected:
    polygon_decomposer::ModifiedTrapezoidal decomposer;
    geometry::Polygon2d poly;

    void SetUp() override
    {
        poly = GetParam().first;
        boost::geometry::correct(poly);

        ASSERT_TRUE(boost::geometry::is_valid(poly));
    }
};

// Core test of this file; given a boost polygon and an expected
// DCEL description, decompose the polygon, check that the produced
// DCEL is valid, and then check that it matches the description
TEST_P(ModifiedTrapezoidalPolygonDecomposition, ProducesCorrectDcel)
{
    const auto dcel = decomposer.decomposePolygon(poly);
    ASSERT_TRUE(DcelIsValid(dcel));

    std::cerr << "Regions identified:" << std::endl;
    for (const auto& region : dcel.regions())
    {
        std::cerr << "\t";
        const auto first = region.edge();
        auto curr        = first;
        do
        {
            std::cerr << to_string(curr.origin().point()) << " ";
            curr = curr.next();
        } while (curr != first);
        std::cerr << std::endl;
    }

    EXPECT_TRUE(DcelMatchesDescription(dcel, GetParam().second));
}

/////////////////////////////////////////////////////////////////////
// Everything below this is generating test cases
/////////////////////////////////////////////////////////////////////

geometry::Ring2d centerOnOrigin(const geometry::Ring2d& ring)
{
    geometry::Point2d centroid(0, 0);

    for (auto it = ring.begin(); it != ring.end() - 1; it++)
        bg::add_point(centroid, *it);
    bg::divide_value(centroid, ring.size() - 1);

    const bg::strategy::transform::translate_transformer<double, 2, 2> center(-centroid.x(), -centroid.y());

    geometry::Ring2d centered;
    bg::transform(ring, centered, center);

    return centered;
}

// Basic Shapes; defined such that
//      The left-bottom-most point is index 0
//      The right-top-most point is index n/2
const geometry::Ring2d SQUARE             = {{0, 0}, {0, 1}, {1, 1}, {1, 0}};
const geometry::Ring2d DIAMOND            = {{0, 0}, {1, 1}, {2, 0}, {1, -1}};
const geometry::Ring2d HEXAGON_HORIZONTAL = {{0, 0}, {1, 1}, {2, 1}, {3, 0}, {2, -1}, {1, -1}};
const geometry::Ring2d HEXAGON_VERTICAL   = {{-1, 1}, {-1, 2}, {0, 3}, {1, 2}, {1, 1}, {0, 0}};
const geometry::Ring2d OCTOGON            = {{0, 0}, {0, 1}, {1, 2}, {2, 2}, {3, 1}, {3, 0}, {2, -1}, {1, -1}};
const geometry::Ring2d DODECAHEDRON = {{0, 0}, {1, 2}, {2, 3}, {4, 4}, {6, 3}, {7, 2}, {8, 0}, {7, -2}, {6, -3}, {4, -4}, {2, -3}, {1, -2}};

// Basic tests:
//      Tests basic shapes as exterior and internal holes. Shapes are
//      chosen to get a mix of vertical edges and non-vertical edges at varying locations relative
//      to the leftmost side of rings
//
//      Colinear, adjacent edges from the input are expected to be treated as if
//      they are a single edge in the input

geometry::Ring2d addColinearities(const geometry::Ring2d& ring)
{
    geometry::Ring2d newRing;

    bg::for_each_segment(ring, [&](const geometry::ConstReferringSegment2d& s) {
        newRing.push_back(s.first);

        auto midpoint = s.first;
        bg::add_point(midpoint, s.second);
        bg::divide_value(midpoint, 2.0);

        newRing.push_back(midpoint);
    });

    return ring;
}

bg::model::multi_point<geometry::Point2d> intersection(const geometry::Segment2d& segment, const geometry::Ring2d& ring)
{
    bg::model::multi_point<geometry::Point2d> result;

    bg::for_each_segment(ring, [&](const geometry::ConstReferringSegment2d& ringSegment) {
        bg::model::multi_point<geometry::Point2d> tmp;
        bg::intersection(segment, ringSegment, tmp);
        result.insert(result.end(), tmp.begin(), tmp.end());
    });

    std::sort(result.begin(), result.end(), [](const geometry::Point2d& l, const geometry::Point2d& r) {
        if (std::abs(l.x() - r.x()) < 0.0001)
            return l.y() < r.y();
        return l.x() < r.x();
    });

    result.erase(std::unique(result.begin(), result.end(),
                             [](const geometry::Point2d& p1, const geometry::Point2d& p2) { return bg::distance(p1, p2) < 0.0001; }),
                 result.end());

    return result;
}
// Tests for a single hole where the points are at various locations relative to the points on the exterior

const std::pair<geometry::Point2d, geometry::Point2d> verticalIntersectionsAt(const geometry::Point2d& target, const geometry::Ring2d& loop)
{
    const geometry::Segment2d verticalLine{{target.x(), 100}, {target.x(), -100}};

    auto intersections = intersection(verticalLine, loop);

    if (intersections.size() != 2u)
        throw std::runtime_error("didn't get 2 intersections");

    std::pair<geometry::Point2d, geometry::Point2d> result(intersections[0], intersections[1]);

    if (result.first.y() < result.second.y())
        std::swap(result.first, result.second);
    return result;
}

const PolygonAndDcel singleHoleCase(const geometry::Ring2d& outer, const geometry::Ring2d& inner, bool outerColinearities,
                                    bool innerColinearities)
{
    PolygonAndDcel result;

    // Scale up outer shape so inner shape fits inside
    bg::strategy::transform::scale_transformer<double, 2, 2> scaleUp(10);
    geometry::Ring2d scaledOuter;
    bg::transform(outer, scaledOuter, scaleUp);

    const auto transformedOuter = centerOnOrigin(scaledOuter);
    const auto transformedInner = centerOnOrigin(inner);

    result.first = {outerColinearities ? addColinearities(transformedOuter) : transformedOuter,
                    {innerColinearities ? addColinearities(transformedInner) : transformedInner}};

    const auto holeStartIntersections = verticalIntersectionsAt(transformedInner[0], transformedOuter);
    const auto holeEndIntersections   = verticalIntersectionsAt(transformedInner[transformedInner.size() / 2], transformedOuter);

    const double innerLeft  = transformedInner[0].x();
    const double innerRight = transformedInner[transformedInner.size() / 2].x();

    // Split the inner loop into 4 pieces; left, right, top, bottom
    typedef bg::ever_circling_iterator<geometry::Ring2d::const_reverse_iterator> circle_it_type;
    circle_it_type left(transformedInner.rbegin(), transformedInner.rend()), right = left, top = left, bottom = left;

    while (left->x() < innerRight)
        left++;
    right = left;
    while (std::abs(left->x() - innerRight) < 0.001)
        left++;
    top = std::prev(left);
    while (left->x() > innerLeft)
        left++;

    // Build the 4 regions that will result from decomposing the shape
    auto& regions = result.second.regions;
    regions.resize(4);

    bg::ever_circling_iterator<geometry::Ring2d::const_iterator> outerIt(transformedOuter.begin(), transformedOuter.end());
    auto outerStart = outerIt;

    // Start first region
    while (outerIt->x() < innerLeft)
        regions[0].push_back(*outerIt++);
    regions[0].push_back(holeStartIntersections.first);
    auto leftIt = left;
    do
    {
        regions[0].push_back(*leftIt);
    } while (leftIt++ != bottom);
    regions[0].pop_back();

    // Do top region
    regions[1].push_back(holeStartIntersections.first);
    while (outerIt->x() < innerRight)
        regions[1].push_back(*outerIt++);
    regions[1].push_back(holeEndIntersections.first);
    auto topIt = top;
    do
    {
        regions[1].push_back(*topIt);
    } while (topIt++ != left);

    // Do right region
    regions[3].push_back(holeEndIntersections.first);
    regions[3].push_back(*outerIt++);
    while (outerIt->x() > innerRight)
        regions[3].push_back(*outerIt++);
    regions[3].push_back(holeEndIntersections.second);
    auto rightIt = right;
    do
    {
        regions[3].push_back(*rightIt);
    } while (rightIt++ != top);

    // Do bottom region
    regions[2].push_back(holeEndIntersections.second);
    while (outerIt->x() > innerLeft)
        regions[2].push_back(*outerIt++);
    regions[2].push_back(holeStartIntersections.second);
    auto bottomIt = bottom;
    do
    {
        regions[2].push_back(*bottomIt);
    } while (bottomIt++ != right);

    // Finish left region
    regions[0].push_back(holeStartIntersections.second);
    while (outerIt != outerStart)
        regions[0].push_back(*outerIt++);

    regions[0].pop_back();
    return result;
}

const PolygonAndDcel exteriorLoopCase(const geometry::Ring2d& outer)
{
    PolygonAndDcel result;
    result.first = {outer};

    result.second.regions.emplace_back();
    for (const auto& pt : outer)
        result.second.regions[0].push_back(pt);
    result.second.regions[0].pop_back();
    return result;
}

const std::vector<PolygonAndDcel> permuteCompositions(const std::vector<geometry::Ring2d>& shapes)
{
    std::vector<PolygonAndDcel> cases;

    // Add index just for debugging crashes
    int i = 0;
    for (auto outer : shapes)
    {
        bg::correct(outer);

        cases.push_back(exteriorLoopCase(outer));
        cases.push_back(exteriorLoopCase(addColinearities(outer)));

        int j = 0;
        for (auto inner : shapes)
        {
            bg::correct(inner);

            cases.push_back(singleHoleCase(outer, inner, false, false));
            cases.push_back(singleHoleCase(outer, inner, true, false));
            cases.push_back(singleHoleCase(outer, inner, false, true));
            cases.push_back(singleHoleCase(outer, inner, true, true));
            j++;
        }
        i++;
    }

    return cases;
}

//! \test Test suite of generated cases with basic shapes. Given a list of shapes,
//! each shape is added as a case, and each composition of any two of them is added as
//! a case.
INSTANTIATE_TEST_SUITE_P(CCPPTests_Basic, ModifiedTrapezoidalPolygonDecomposition,
                         ValuesIn(permuteCompositions({SQUARE, DIAMOND, HEXAGON_VERTICAL, HEXAGON_HORIZONTAL, OCTOGON, DODECAHEDRON})));

// Tests for when there are weird convexities on the left side of the outer loop
INSTANTIATE_TEST_SUITE_P(
    CCPPTests_Convexities_Outer_Left, ModifiedTrapezoidalPolygonDecomposition,
    Values(
        //! \test Test decomposition with one single-point convexity on opening side of outer loop
        PolygonAndDcel{{{{3, 0}, {3, 3}, {0, 5}, {3, 7}, {3, 10}, {10, 10}, {10, 0}}},
                       {{{{3, 0}, {3, 3}, {0, 5}, {3, 7}, {3, 10}, {10, 10}, {10, 0}}}}},

        //! \test Test decomposition with one multi-point convexity on opening side of outer loop
        PolygonAndDcel{{{{3, 0}, {3, 2}, {2, 3}, {1, 5}, {2, 7}, {3, 8}, {3, 10}, {10, 10}, {10, 0}}},
                       {{{{3, 0}, {3, 2}, {2, 3}, {1, 5}, {2, 7}, {3, 8}, {3, 10}, {10, 10}, {10, 0}}}}},

        //! \test Test decomposition with one single-point convexity as the opening side of outer loop
        PolygonAndDcel{{{{3, 0}, {0, 5}, {3, 10}, {10, 10}, {10, 0}}}, {{{{3, 0}, {0, 5}, {3, 10}, {10, 10}, {10, 0}}}}},

        //! \test Test decomposition with one multi-point convexity as the opening side of outer loop
        PolygonAndDcel{{{{3, 0}, {1, 1}, {0, 5}, {1, 9}, {3, 10}, {10, 10}, {10, 0}}},
                       {{{{3, 0}, {1, 1}, {0, 5}, {1, 9}, {3, 10}, {10, 10}, {10, 0}}}}},

        //! \test Test decomposition with two single-point convexities on opening side of outer loop
        PolygonAndDcel{
            {{{3, 0}, {3, 2}, {0, 3}, {3, 4}, {3, 6}, {0, 7}, {3, 8}, {3, 10}, {10, 10}, {10, 0}}},
            {{{{3, 2}, {0, 3}, {3, 4}}, {{3, 6}, {0, 7}, {3, 8}}, {{3, 0}, {3, 2}, {3, 4}, {3, 6}, {3, 8}, {3, 10}, {10, 10}, {10, 0}}}}},

        //! \test Test decomposition with two multi-point convexities on opening side of outer loop
        PolygonAndDcel{{{{3, 0},
                         {3, 2},
                         {2, 2.25},
                         {0, 3},
                         {2, 3.75},
                         {3, 4},
                         {3, 6},
                         {2, 6.25},
                         {0, 7},
                         {2, 7.75},
                         {3, 8},
                         {3, 10},
                         {10, 10},
                         {10, 0}}},
                       {{{{3, 2}, {2, 2.25}, {0, 3}, {2, 3.75}, {3, 4}},
                         {{3, 6}, {2, 6.25}, {0, 7}, {2, 7.75}, {3, 8}},
                         {{3, 0}, {3, 2}, {3, 4}, {3, 6}, {3, 8}, {3, 10}, {10, 10}, {10, 0}}}}},

        //! \test Test decomposition with two single-point convexities as the opening side of outer loop
        PolygonAndDcel{{{{3, 0}, {1, 3}, {3, 5}, {1, 7}, {3, 10}, {10, 10}, {10, 0}}},
                       {{{{3, 0}, {1, 3}, {3, 5}}, {{3, 5}, {1, 7}, {3, 10}}, {{3, 0}, {3, 5}, {3, 10}, {10, 10}, {10, 0}}}}},

        //! \test Test decomposition with two multi-point convexities as the opening side of outer loop
        PolygonAndDcel{{{{3, 0}, {2, 1}, {1, 3}, {2, 3.75}, {3, 5}, {2, 5.25}, {1, 7}, {2, 9}, {3, 10}, {10, 10}, {10, 0}}},
                       {{{{3, 0}, {2, 1}, {1, 3}, {2, 3.75}, {3, 5}},
                         {{3, 5}, {2, 5.25}, {1, 7}, {2, 9}, {3, 10}},
                         {{3, 10}, {10, 10}, {10, 0}, {3, 0}, {3, 5}}}}}

        ));

// Tests for when there are weird convexities on the left side of the inner loop
INSTANTIATE_TEST_SUITE_P(
    CCPPTests_Convexities_Inner_Left, ModifiedTrapezoidalPolygonDecomposition,
    Values(
        //! \test Test decomposition with o single-point convexity on opening side of inner loop
        PolygonAndDcel{{{{0, 0}, {0, 10}, {10, 10}, {10, 0}}, {{{4, 2}, {4, 4}, {3, 5}, {4, 6}, {4, 8}, {8, 8}, {8, 2}}}},
                       {{{{0, 0}, {0, 10}, {3, 10}, {3, 5}, {3, 0}},
                         {{3, 10}, {8, 10}, {8, 8}, {4, 8}, {4, 6}, {3, 5}},
                         {{3, 0}, {3, 5}, {4, 4}, {4, 2}, {8, 2}, {8, 0}},
                         {{8, 0}, {8, 2}, {8, 8}, {8, 10}, {10, 10}, {10, 0}}}}},

        //! \test Test decomposition with o multi-point convexity on opening side of inner loop
        PolygonAndDcel{
            {{{0, 0}, {0, 10}, {10, 10}, {10, 0}}, {{{4, 2}, {4, 4}, {3, 4.25}, {2, 5}, {3, 5.75}, {4, 6}, {4, 8}, {8, 8}, {8, 2}}}},
            {{{{0, 0}, {0, 10}, {2, 10}, {2, 5}, {2, 0}},
              {{2, 10}, {8, 10}, {8, 8}, {4, 8}, {4, 6}, {3, 5.75}, {2, 5}},
              {{2, 0}, {2, 5}, {3, 4.25}, {4, 4}, {4, 2}, {8, 2}, {8, 0}},
              {{8, 10}, {10, 10}, {10, 0}, {8, 0}, {8, 2}, {8, 8}}}}},

        //! \test Test decomposition with o single-point convexity as the opening side of inner loop
        PolygonAndDcel{{{{0, 0}, {0, 10}, {10, 10}, {10, 0}}, {{{4, 2}, {3, 5}, {4, 8}, {8, 8}, {8, 2}}}},
                       {{{{0, 0}, {0, 10}, {3, 10}, {3, 5}, {3, 0}},
                         {{3, 5}, {3, 10}, {8, 10}, {8, 8}, {4, 8}},
                         {{3, 5}, {4, 2}, {8, 2}, {8, 0}, {3, 0}},
                         {{8, 10}, {10, 10}, {10, 0}, {8, 0}, {8, 2}, {8, 8}}}}},

        //! \test Test decomposition with o multi-point convexity as the opening side of inner loop
        PolygonAndDcel{{{{0, 0}, {0, 10}, {10, 10}, {10, 0}}, {{{4, 2}, {3, 3}, {2, 5}, {3, 7}, {4, 8}, {8, 8}, {8, 2}}}},
                       {{{{0, 0}, {0, 10}, {2, 10}, {2, 5}, {2, 0}},
                         {{2, 5}, {2, 10}, {8, 10}, {8, 8}, {4, 8}, {3, 7}},
                         {{2, 5}, {3, 3}, {4, 2}, {8, 2}, {8, 0}, {2, 0}},
                         {{8, 8}, {8, 10}, {10, 10}, {10, 0}, {8, 0}, {8, 2}}}}},

        //! \test Test decomposition with t single-point convexities on opening side of inner loop
        PolygonAndDcel{
            {{{0, 0}, {0, 10}, {10, 10}, {10, 0}}, {{{4, 2}, {4, 3}, {3, 3.5}, {4, 4}, {4, 6}, {3, 6.5}, {4, 7}, {4, 8}, {8, 8}, {8, 2}}}},
            {{{{0, 0}, {0, 10}, {3, 10}, {3, 6.5}, {3, 3.5}, {3, 0}},
              {{3, 6.5}, {4, 6}, {4, 4}, {3, 3.5}},
              {{3, 10}, {8, 10}, {8, 8}, {4, 8}, {4, 7}, {3, 6.5}},
              {{3, 0}, {3, 3.5}, {4, 3}, {4, 2}, {8, 2}, {8, 0}},
              {{8, 0}, {8, 2}, {8, 8}, {8, 10}, {10, 10}, {10, 0}}}}},

        //! \test Test decomposition with t multi-point convexities on opening side of inner loop
        PolygonAndDcel{{{{0, 0}, {0, 10}, {10, 10}, {10, 0}},
                        {{{4, 2},
                          {4, 3},
                          {3.75, 3.25},
                          {2, 3.5},
                          {3.75, 3.75},
                          {4, 4},
                          {4, 6},
                          {3.75, 6.25},
                          {2, 6.5},
                          {3.75, 6.75},
                          {4, 7},
                          {4, 8},
                          {8, 8},
                          {8, 2}}}},
                       {{{{0, 0}, {0, 10}, {2, 10}, {2, 6.5}, {2, 3.5}, {2, 0}},
                         {{2, 3.5}, {2, 6.5}, {3.75, 6.25}, {4, 6}, {4, 4}, {3.75, 3.75}},
                         {{2, 6.5}, {2, 10}, {8, 10}, {8, 8}, {4, 8}, {4, 7}, {3.75, 6.75}},
                         {{2, 3.5}, {3.75, 3.25}, {4, 3}, {4, 2}, {8, 2}, {8, 0}, {2, 0}},
                         {{8, 10}, {10, 10}, {10, 0}, {8, 0}, {8, 2}, {8, 8}}}}},

        //! \test Test decomposition with t single-point convexities as the opening side of inner loop
        PolygonAndDcel{{{{0, 0}, {0, 10}, {10, 10}, {10, 0}}, {{{4, 2}, {3, 4}, {4, 5}, {3, 6}, {4, 8}, {8, 8}, {8, 2}}}},
                       {{{{0, 0}, {0, 10}, {3, 10}, {3, 6}, {3, 4}, {3, 0}},
                         {{3, 6}, {4, 5}, {3, 4}},
                         {{3, 6}, {3, 10}, {8, 10}, {8, 8}, {4, 8}},
                         {{3, 4}, {4, 2}, {8, 2}, {8, 0}, {3, 0}},
                         {{8, 10}, {10, 10}, {10, 0}, {8, 0}, {8, 2}, {8, 8}}}}},

        //! \test Test decomposition with t multi-point convexities as the opening side of inner loop
        PolygonAndDcel{{{{0, 0}, {0, 10}, {10, 10}, {10, 0}},
                        {{{4, 2}, {3, 2.5}, {2, 4}, {3, 4.75}, {4, 5}, {3, 5.25}, {2, 6}, {3, 7.5}, {4, 8}, {8, 8}, {8, 2}}}},
                       {{{{0, 0}, {0, 10}, {2, 10}, {2, 6}, {2, 4}, {2, 0}},
                         {{2, 4}, {2, 6}, {3, 5.25}, {4, 5}, {3, 4.75}},
                         {{2, 4}, {3, 2.5}, {4, 2}, {8, 2}, {8, 0}, {2, 0}},
                         {{2, 6}, {2, 10}, {8, 10}, {8, 8}, {4, 8}, {3, 7.5}},
                         {{8, 10}, {10, 10}, {10, 0}, {8, 0}, {8, 2}, {8, 8}}}}}

        ));

// Tests for when there are weird convexities on the right side of the outer loop
INSTANTIATE_TEST_SUITE_P(
    CCPPTests_Convexities_Outer_Right, ModifiedTrapezoidalPolygonDecomposition,
    Values(
        //! \test Test decomposition with o single-point convexity on closing side of outer loop
        PolygonAndDcel{{{{-3, 0}, {-3, -3}, {0, -5}, {-3, -7}, {-3, -10}, {-10, -10}, {-10, 0}}},
                       {{{{-3, 0}, {-3, -3}, {0, -5}, {-3, -7}, {-3, -10}, {-10, -10}, {-10, 0}}}}},

        //! \test Test decomposition with o multi-point convexity on closing side of outer loop
        PolygonAndDcel{{{{-3, 0}, {-3, -2}, {-2, -3}, {-1, -5}, {-2, -7}, {-3, -8}, {-3, -10}, {-10, -10}, {-10, 0}}},
                       {{{{-3, 0}, {-3, -2}, {-2, -3}, {-1, -5}, {-2, -7}, {-3, -8}, {-3, -10}, {-10, -10}, {-10, 0}}}}},

        //! \test Test decomposition with o single-point convexity as the closing side of outer loop
        PolygonAndDcel{{{{-3, 0}, {0, -5}, {-3, -10}, {-10, -10}, {-10, 0}}}, {{{{-3, 0}, {0, -5}, {-3, -10}, {-10, -10}, {-10, 0}}}}},

        //! \test Test decomposition with o multi-point convexity as the closing side of outer loop
        PolygonAndDcel{{{{-3, 0}, {-1, -1}, {0, -5}, {-1, -9}, {-3, -10}, {-10, -10}, {-10, 0}}},
                       {{{{-3, 0}, {-1, -1}, {0, -5}, {-1, -9}, {-3, -10}, {-10, -10}, {-10, 0}}}}},

        //! \test Test decomposition with t single-point convexities on closing side of outer loop
        PolygonAndDcel{{{{-3, 0}, {-3, -2}, {0, -3}, {-3, -4}, {-3, -6}, {0, -7}, {-3, -8}, {-3, -10}, {-10, -10}, {-10, 0}}},
                       {{{{-3, -2}, {0, -3}, {-3, -4}},
                         {{-3, -6}, {0, -7}, {-3, -8}},
                         {{-3, 0}, {-3, -2}, {-3, -4}, {-3, -6}, {-3, -8}, {-3, -10}, {-10, -10}, {-10, 0}}}}},

        //! \test Test decomposition with t multi-point convexities on closing side of outer loop
        PolygonAndDcel{{{{-3, 0},
                         {-3, -2},
                         {-2, -2.25},
                         {0, -3},
                         {-2, -3.75},
                         {-3, -4},
                         {-3, -6},
                         {-2, -6.25},
                         {0, -7},
                         {-2, -7.75},
                         {-3, -8},
                         {-3, -10},
                         {-10, -10},
                         {-10, 0}}},
                       {{{{-3, -2}, {-2, -2.25}, {0, -3}, {-2, -3.75}, {-3, -4}},
                         {{-3, -6}, {-2, -6.25}, {0, -7}, {-2, -7.75}, {-3, -8}},
                         {{-3, 0}, {-3, -2}, {-3, -4}, {-3, -6}, {-3, -8}, {-3, -10}, {-10, -10}, {-10, 0}}}}},

        //! \test Test decomposition with t single-point convexities as the closing side of outer loop
        PolygonAndDcel{
            {{{-3, 0}, {-1, -3}, {-3, -5}, {-1, -7}, {-3, -10}, {-10, -10}, {-10, 0}}},
            {{{{-3, 0}, {-1, -3}, {-3, -5}}, {{-3, -5}, {-1, -7}, {-3, -10}}, {{-3, 0}, {-3, -5}, {-3, -10}, {-10, -10}, {-10, 0}}}}},

        //! \test Test decomposition with t multi-point convexities as the closing side of outer loop
        PolygonAndDcel{
            {{{-3, 0}, {-2, -1}, {-1, -3}, {-2, -3.75}, {-3, -5}, {-2, -5.25}, {-1, -7}, {-2, -9}, {-3, -10}, {-10, -10}, {-10, 0}}},
            {{{{-3, 0}, {-2, -1}, {-1, -3}, {-2, -3.75}, {-3, -5}},
              {{-3, -5}, {-2, -5.25}, {-1, -7}, {-2, -9}, {-3, -10}},
              {{-3, -10}, {-10, -10}, {-10, 0}, {-3, 0}, {-3, -5}}}}}

        ));

// Tests for when there are weird convexities on the right side of the inner loop
INSTANTIATE_TEST_SUITE_P(
    CCPPTests_Convexities_Inner_Right, ModifiedTrapezoidalPolygonDecomposition,
    Values(
        //! \test Test decomposition with o single-point convexity on closing side of inner loop
        PolygonAndDcel{{{{0, 0}, {0, -10}, {-10, -10}, {-10, 0}}, {{{-4, -2}, {-4, -4}, {-3, -5}, {-4, -6}, {-4, -8}, {-8, -8}, {-8, -2}}}},
                       {{{{0, 0}, {0, -10}, {-3, -10}, {-3, -5}, {-3, 0}},
                         {{-3, -10}, {-8, -10}, {-8, -8}, {-4, -8}, {-4, -6}, {-3, -5}},
                         {{-3, 0}, {-3, -5}, {-4, -4}, {-4, -2}, {-8, -2}, {-8, 0}},
                         {{-8, 0}, {-8, -2}, {-8, -8}, {-8, -10}, {-10, -10}, {-10, 0}}}}},

        //! \test Test decomposition with o multi-point convexity on closing side of inner loop
        PolygonAndDcel{{{{0, 0}, {0, -10}, {-10, -10}, {-10, 0}},
                        {{{-4, -2}, {-4, -4}, {-3, -4.25}, {-2, -5}, {-3, -5.75}, {-4, -6}, {-4, -8}, {-8, -8}, {-8, -2}}}},
                       {{{{0, 0}, {0, -10}, {-2, -10}, {-2, -5}, {-2, 0}},
                         {{-2, -10}, {-8, -10}, {-8, -8}, {-4, -8}, {-4, -6}, {-3, -5.75}, {-2, -5}},
                         {{-2, 0}, {-2, -5}, {-3, -4.25}, {-4, -4}, {-4, -2}, {-8, -2}, {-8, 0}},
                         {{-8, -10}, {-10, -10}, {-10, 0}, {-8, 0}, {-8, -2}, {-8, -8}}}}},

        //! \test Test decomposition with o single-point convexity as the closing side of inner loop
        PolygonAndDcel{{{{0, 0}, {0, -10}, {-10, -10}, {-10, 0}}, {{{-4, -2}, {-3, -5}, {-4, -8}, {-8, -8}, {-8, -2}}}},
                       {{{{0, 0}, {0, -10}, {-3, -10}, {-3, -5}, {-3, 0}},
                         {{-3, -5}, {-3, -10}, {-8, -10}, {-8, -8}, {-4, -8}},
                         {{-3, -5}, {-4, -2}, {-8, -2}, {-8, 0}, {-3, 0}},
                         {{-8, -10}, {-10, -10}, {-10, 0}, {-8, 0}, {-8, -2}, {-8, -8}}}}},

        //! \test Test decomposition with o multi-point convexity as the closing side of inner loop
        PolygonAndDcel{{{{0, 0}, {0, -10}, {-10, -10}, {-10, 0}}, {{{-4, -2}, {-3, -3}, {-2, -5}, {-3, -7}, {-4, -8}, {-8, -8}, {-8, -2}}}},
                       {{{{0, 0}, {0, -10}, {-2, -10}, {-2, -5}, {-2, 0}},
                         {{-2, -5}, {-2, -10}, {-8, -10}, {-8, -8}, {-4, -8}, {-3, -7}},
                         {{-2, -5}, {-3, -3}, {-4, -2}, {-8, -2}, {-8, 0}, {-2, 0}},
                         {{-8, -8}, {-8, -10}, {-10, -10}, {-10, 0}, {-8, 0}, {-8, -2}}}}},

        //! \test Test decomposition with t single-point convexities on closing side of inner loop
        PolygonAndDcel{{{{0, 0}, {0, -10}, {-10, -10}, {-10, 0}},
                        {{{-4, -2}, {-4, -3}, {-3, -3.5}, {-4, -4}, {-4, -6}, {-3, -6.5}, {-4, -7}, {-4, -8}, {-8, -8}, {-8, -2}}}},
                       {{{{0, 0}, {0, -10}, {-3, -10}, {-3, -6.5}, {-3, -3.5}, {-3, 0}},
                         {{-3, -6.5}, {-4, -6}, {-4, -4}, {-3, -3.5}},
                         {{-3, -10}, {-8, -10}, {-8, -8}, {-4, -8}, {-4, -7}, {-3, -6.5}},
                         {{-3, 0}, {-3, -3.5}, {-4, -3}, {-4, -2}, {-8, -2}, {-8, 0}},
                         {{-8, 0}, {-8, -2}, {-8, -8}, {-8, -10}, {-10, -10}, {-10, 0}}}}},

        //! \test Test decomposition with t multi-point convexities on closing side of inner loop
        PolygonAndDcel{{{{0, 0}, {0, -10}, {-10, -10}, {-10, 0}},
                        {{{-4, -2},
                          {-4, -3},
                          {-3.75, -3.25},
                          {-2, -3.5},
                          {-3.75, -3.75},
                          {-4, -4},
                          {-4, -6},
                          {-3.75, -6.25},
                          {-2, -6.5},
                          {-3.75, -6.75},
                          {-4, -7},
                          {-4, -8},
                          {-8, -8},
                          {-8, -2}}}},
                       {{{{0, 0}, {0, -10}, {-2, -10}, {-2, -6.5}, {-2, -3.5}, {-2, 0}},
                         {{-2, -3.5}, {-2, -6.5}, {-3.75, -6.25}, {-4, -6}, {-4, -4}, {-3.75, -3.75}},
                         {{-2, -6.5}, {-2, -10}, {-8, -10}, {-8, -8}, {-4, -8}, {-4, -7}, {-3.75, -6.75}},
                         {{-2, -3.5}, {-3.75, -3.25}, {-4, -3}, {-4, -2}, {-8, -2}, {-8, 0}, {-2, 0}},
                         {{-8, -10}, {-10, -10}, {-10, 0}, {-8, 0}, {-8, -2}, {-8, -8}}}}},

        //! \test Test decomposition with t single-point convexities as the closing side of inner loop
        PolygonAndDcel{{{{0, 0}, {0, -10}, {-10, -10}, {-10, 0}}, {{{-4, -2}, {-3, -4}, {-4, -5}, {-3, -6}, {-4, -8}, {-8, -8}, {-8, -2}}}},
                       {{{{0, 0}, {0, -10}, {-3, -10}, {-3, -6}, {-3, -4}, {-3, 0}},
                         {{-3, -6}, {-4, -5}, {-3, -4}},
                         {{-3, -6}, {-3, -10}, {-8, -10}, {-8, -8}, {-4, -8}},
                         {{-3, -4}, {-4, -2}, {-8, -2}, {-8, 0}, {-3, 0}},
                         {{-8, -10}, {-10, -10}, {-10, 0}, {-8, 0}, {-8, -2}, {-8, -8}}}}},

        //! \test Test decomposition with t multi-point convexities as the closing side of inner loop
        PolygonAndDcel{
            {{{0, 0}, {0, -10}, {-10, -10}, {-10, 0}},
             {{{-4, -2}, {-3, -2.5}, {-2, -4}, {-3, -4.75}, {-4, -5}, {-3, -5.25}, {-2, -6}, {-3, -7.5}, {-4, -8}, {-8, -8}, {-8, -2}}}},
            {{{{0, 0}, {0, -10}, {-2, -10}, {-2, -6}, {-2, -4}, {-2, 0}},
              {{-2, -4}, {-2, -6}, {-3, -5.25}, {-4, -5}, {-3, -4.75}},
              {{-2, -4}, {-3, -2.5}, {-4, -2}, {-8, -2}, {-8, 0}, {-2, 0}},
              {{-2, -6}, {-2, -10}, {-8, -10}, {-8, -8}, {-4, -8}, {-3, -7.5}},
              {{-8, -10}, {-10, -10}, {-10, 0}, {-8, 0}, {-8, -2}, {-8, -8}}}}}

        ));

// Tests for when there are weird concavities on the left side of the outer loop
INSTANTIATE_TEST_SUITE_P(
    CCPPTests_Concavities_Outer_Left, ModifiedTrapezoidalPolygonDecomposition,
    Values(
        //! \test Test decomposition with o single-point concavity on opening side of outer loop
        PolygonAndDcel{
            {{{0, 0}, {0, 3}, {2, 5}, {0, 7}, {0, 10}, {10, 10}, {10, 0}}},
            {{{{0, 0}, {0, 3}, {2, 5}, {2, 0}}, {{0, 7}, {0, 10}, {2, 10}, {2, 5}}, {{2, 5}, {2, 10}, {10, 10}, {10, 0}, {2, 0}}}}},

        //! \test Test decomposition with o multi-point concavity on opening side of outer loop
        PolygonAndDcel{{{{0, 0}, {0, 2}, {1, 3}, {2, 5}, {1, 7}, {0, 8}, {0, 10}, {10, 10}, {10, 0}}},
                       {{{{0, 0}, {0, 2}, {1, 3}, {2, 5}, {2, 0}},
                         {{0, 8}, {0, 10}, {2, 10}, {2, 5}, {1, 7}},
                         {{2, 5}, {2, 10}, {10, 10}, {10, 0}, {2, 0}}}}},

        //! \test Test decomposition with o single-point concavity as the opening side of outer loop
        PolygonAndDcel{{{{0, 0}, {2, 5}, {0, 10}, {10, 10}, {10, 0}}},
                       {{{{0, 0}, {2, 5}, {2, 0}}, {{0, 10}, {2, 10}, {2, 5}}, {{2, 5}, {2, 10}, {10, 10}, {10, 0}, {2, 0}}}}},

        //! \test Test decomposition with o multi-point concavity as the opening side of outer loop
        PolygonAndDcel{
            {{{0, 0}, {1, 1}, {2, 5}, {1, 9}, {0, 10}, {10, 10}, {10, 0}}},
            {{{{0, 0}, {1, 1}, {2, 5}, {2, 0}}, {{0, 10}, {2, 10}, {2, 5}, {1, 9}}, {{2, 5}, {2, 10}, {10, 10}, {10, 0}, {2, 0}}}}},

        //! \test Test decomposition with t single-point concavities on opening side of outer loop
        PolygonAndDcel{{{{0, 0}, {0, 2}, {2, 3}, {0, 4}, {0, 6}, {2, 7}, {0, 8}, {0, 10}, {10, 10}, {10, 0}}},
                       {{{{0, 0}, {0, 2}, {2, 3}, {2, 0}},
                         {{2, 3}, {0, 4}, {0, 6}, {2, 7}},
                         {{0, 8}, {0, 10}, {2, 10}, {2, 7}},
                         {{2, 3}, {2, 7}, {2, 10}, {10, 10}, {10, 0}, {2, 0}}}}},

        //! \test Test decomposition with t multi-point concavities on opening side of outer loop
        PolygonAndDcel{{{{0, 0},
                         {0, 2},
                         {1, 2.25},
                         {2, 3},
                         {1, 3.75},
                         {0, 4},
                         {0, 6},
                         {1, 6.25},
                         {2, 7},
                         {1, 7.75},
                         {0, 8},
                         {0, 10},
                         {10, 10},
                         {10, 0}}},
                       {{{{0, 0}, {0, 2}, {1, 2.25}, {2, 3}, {2, 0}},
                         {{2, 3}, {1, 3.75}, {0, 4}, {0, 6}, {1, 6.25}, {2, 7}},
                         {{0, 8}, {0, 10}, {2, 10}, {2, 7}, {1, 7.75}},
                         {{2, 3}, {2, 7}, {2, 10}, {10, 10}, {10, 0}, {2, 0}}}}},

        //! \test Test decomposition with t single-point concavities as the opening side of outer loop
        PolygonAndDcel{{{{0, 0}, {2, 3}, {0, 5}, {2, 7}, {0, 10}, {10, 10}, {10, 0}}},
                       {{{{0, 0}, {2, 3}, {2, 0}},
                         {{2, 3}, {0, 5}, {2, 7}},
                         {{0, 10}, {2, 10}, {2, 7}},
                         {{2, 3}, {2, 7}, {2, 10}, {10, 10}, {10, 0}, {2, 0}}}}},

        //! \test Test decomposition with t multi-point concavities as the opening side of outer loop
        PolygonAndDcel{{{{0, 0}, {1, 1}, {2, 3}, {1, 3.75}, {0, 5}, {1, 5.25}, {2, 7}, {1, 9}, {0, 10}, {10, 10}, {10, 0}}},
                       {{{{0, 0}, {1, 1}, {2, 3}, {2, 0}},
                         {{2, 3}, {1, 3.75}, {0, 5}, {1, 5.25}, {2, 7}},
                         {{0, 10}, {2, 10}, {2, 7}, {1, 9}},
                         {{2, 3}, {2, 7}, {2, 10}, {10, 10}, {10, 0}, {2, 0}}}}}

        ));

// Tests for when there are weird concavities on the left side of the inner loop
INSTANTIATE_TEST_SUITE_P(
    CCPPTests_Concavities_Inner_Left, ModifiedTrapezoidalPolygonDecomposition,
    Values(
        //! \test Test decomposition with o single-point concavity on opening side of inner loop
        PolygonAndDcel{{{{0, 0}, {0, 10}, {10, 10}, {10, 0}}, {{{2, 2}, {2, 4}, {3, 5}, {2, 6}, {2, 8}, {8, 8}, {8, 2}}}},
                       {{{{0, 0}, {0, 10}, {2, 10}, {2, 8}, {2, 6}, {2, 4}, {2, 2}, {2, 0}},
                         {{2, 0}, {2, 2}, {8, 2}, {8, 0}},
                         {{2, 4}, {2, 6}, {3, 5}},
                         {{2, 8}, {2, 10}, {8, 10}, {8, 8}},
                         {{8, 10}, {10, 10}, {10, 0}, {8, 0}, {8, 2}, {8, 8}}}}},

        //! \test Test decomposition with o multi-point concavity on opening side of inner loop
        PolygonAndDcel{
            {{{0, 0}, {0, 10}, {10, 10}, {10, 0}}, {{{2, 2}, {2, 4}, {2.5, 4.25}, {3, 5}, {2.5, 5.75}, {2, 6}, {2, 8}, {8, 8}, {8, 2}}}},
            {{{{0, 0}, {0, 10}, {2, 10}, {2, 8}, {2, 6}, {2, 4}, {2, 2}, {2, 0}},
              {{2, 0}, {2, 2}, {8, 2}, {8, 0}},
              {{2, 4}, {2, 6}, {2.5, 5.75}, {3, 5}, {2.5, 4.25}},
              {{2, 8}, {2, 10}, {8, 10}, {8, 8}},
              {{8, 10}, {10, 10}, {10, 0}, {8, 0}, {8, 2}, {8, 8}}}}},

        //! \test Test decomposition with o single-point concavity as the opening side of inner loop
        PolygonAndDcel{{{{0, 0}, {0, 10}, {10, 10}, {10, 0}}, {{{2, 2}, {3, 5}, {2, 8}, {8, 8}, {8, 2}}}},
                       {{{{0, 0}, {0, 10}, {2, 10}, {2, 8}, {2, 2}, {2, 0}},
                         {{2, 0}, {2, 2}, {8, 2}, {8, 0}},
                         {{2, 2}, {2, 8}, {3, 5}},
                         {{2, 8}, {2, 10}, {8, 10}, {8, 8}},
                         {{8, 10}, {10, 10}, {10, 0}, {8, 0}, {8, 2}, {8, 8}}}}},

        //! \test Test decomposition with o multi-point concavity as the opening side of inner loop
        PolygonAndDcel{{{{0, 0}, {0, 10}, {10, 10}, {10, 0}}, {{{2, 2}, {3, 3}, {4, 5}, {3, 7}, {2, 8}, {8, 8}, {8, 2}}}},
                       {{{{0, 0}, {0, 10}, {2, 10}, {2, 8}, {2, 2}, {2, 0}},
                         {{2, 0}, {2, 2}, {8, 2}, {8, 0}},
                         {{2, 2}, {2, 8}, {3, 7}, {4, 5}, {3, 3}},
                         {{2, 8}, {2, 10}, {8, 10}, {8, 8}},
                         {{8, 10}, {10, 10}, {10, 0}, {8, 0}, {8, 2}, {8, 8}}}}},

        //! \test Test decomposition with t single-point concavities on opening side of inner loop
        PolygonAndDcel{
            {{{0, 0}, {0, 10}, {10, 10}, {10, 0}}, {{{2, 2}, {2, 3}, {3, 3.5}, {2, 4}, {2, 6}, {3, 6.5}, {2, 7}, {2, 8}, {8, 8}, {8, 2}}}},
            {{{{0, 0}, {0, 10}, {2, 10}, {2, 8}, {2, 7}, {2, 6}, {2, 4}, {2, 3}, {2, 2}, {2, 0}},
              {{2, 0}, {2, 2}, {8, 2}, {8, 0}},
              {{2, 3}, {2, 4}, {3, 3.5}},
              {{2, 6}, {2, 7}, {3, 6.5}},
              {{2, 8}, {2, 10}, {8, 10}, {8, 8}},
              {{8, 10}, {10, 10}, {10, 0}, {8, 0}, {8, 2}, {8, 8}}}}},

        //! \test Test decomposition with t multi-point concavities on opening side of inner loop
        PolygonAndDcel{{{{0, 0}, {0, 10}, {10, 10}, {10, 0}},
                        {{{2, 2},
                          {2, 3},
                          {3.75, 3.25},
                          {4, 3.5},
                          {3.75, 3.75},
                          {2, 4},
                          {2, 6},
                          {3.75, 6.25},
                          {4, 6.5},
                          {3.75, 6.75},
                          {2, 7},
                          {2, 8},
                          {8, 8},
                          {8, 2}}}},
                       {{{{0, 0}, {0, 10}, {2, 10}, {2, 8}, {2, 7}, {2, 6}, {2, 4}, {2, 3}, {2, 2}, {2, 0}},
                         {{2, 0}, {2, 2}, {8, 2}, {8, 0}},
                         {{2, 3}, {2, 4}, {3.75, 3.75}, {4, 3.5}, {3.75, 3.25}},
                         {{2, 6}, {2, 7}, {3.75, 6.75}, {4, 6.5}, {3.75, 6.25}},
                         {{2, 8}, {2, 10}, {8, 10}, {8, 8}},
                         {{8, 10}, {10, 10}, {10, 0}, {8, 0}, {8, 2}, {8, 8}}}}},

        //! \test Test decomposition with t single-point concavities as the opening side of inner loop
        PolygonAndDcel{{{{0, 0}, {0, 10}, {10, 10}, {10, 0}}, {{{2, 2}, {3, 4}, {2, 5}, {3, 6}, {2, 8}, {8, 8}, {8, 2}}}},
                       {{{{0, 0}, {0, 10}, {2, 10}, {2, 8}, {2, 5}, {2, 2}, {2, 0}},
                         {{2, 0}, {2, 2}, {8, 2}, {8, 0}},
                         {{2, 2}, {2, 5}, {3, 4}},
                         {{2, 5}, {2, 8}, {3, 6}},
                         {{2, 8}, {2, 10}, {8, 10}, {8, 8}},
                         {{8, 10}, {10, 10}, {10, 0}, {8, 0}, {8, 2}, {8, 8}}}}},

        //! \test Test decomposition with t multi-point concavities as the opening side of inner loop
        PolygonAndDcel{{{{0, 0}, {0, 10}, {10, 10}, {10, 0}},
                        {{{2, 2}, {3, 2.5}, {4, 4}, {3, 4.75}, {2, 5}, {3, 5.25}, {4, 6}, {3, 7.5}, {2, 8}, {8, 8}, {8, 2}}}},
                       {{{{0, 0}, {0, 10}, {2, 10}, {2, 8}, {2, 5}, {2, 2}, {2, 0}},
                         {{2, 0}, {2, 2}, {8, 2}, {8, 0}},
                         {{2, 2}, {2, 5}, {3, 4.75}, {4, 4}, {3, 2.5}},
                         {{2, 5}, {2, 8}, {3, 7.5}, {4, 6}, {3, 5.25}},
                         {{2, 8}, {2, 10}, {8, 10}, {8, 8}},
                         {{8, 10}, {10, 10}, {10, 0}, {8, 0}, {8, 2}, {8, 8}}}}}

        ));

// Tests for when there are weird concavities on the right side of the outer loop
INSTANTIATE_TEST_SUITE_P(
    CCPPTests_Concavities_Outer_Right, ModifiedTrapezoidalPolygonDecomposition,
    Values(
        //! \test Test decomposition with o single-point concavity on closing side of outer loop
        PolygonAndDcel{{{{0, 0}, {0, -3}, {-2, -5}, {0, -7}, {0, -10}, {-10, -10}, {-10, 0}}},
                       {{{{0, 0}, {0, -3}, {-2, -5}, {-2, 0}},
                         {{0, -7}, {0, -10}, {-2, -10}, {-2, -5}},
                         {{-2, -5}, {-2, -10}, {-10, -10}, {-10, 0}, {-2, 0}}}}},

        //! \test Test decomposition with o multi-point concavity on closing side of outer loop
        PolygonAndDcel{{{{0, 0}, {0, -2}, {-1, -3}, {-2, -5}, {-1, -7}, {0, -8}, {0, -10}, {-10, -10}, {-10, 0}}},
                       {{{{0, 0}, {0, -2}, {-1, -3}, {-2, -5}, {-2, 0}},
                         {{0, -8}, {0, -10}, {-2, -10}, {-2, -5}, {-1, -7}},
                         {{-2, -5}, {-2, -10}, {-10, -10}, {-10, 0}, {-2, 0}}}}},

        //! \test Test decomposition with o single-point concavity as the closing side of outer loop
        PolygonAndDcel{
            {{{0, 0}, {-2, -5}, {0, -10}, {-10, -10}, {-10, 0}}},
            {{{{0, 0}, {-2, -5}, {-2, 0}}, {{0, -10}, {-2, -10}, {-2, -5}}, {{-2, -5}, {-2, -10}, {-10, -10}, {-10, 0}, {-2, 0}}}}},

        //! \test Test decomposition with o multi-point concavity as the closing side of outer loop
        PolygonAndDcel{{{{0, 0}, {-1, -1}, {-2, -5}, {-1, -9}, {0, -10}, {-10, -10}, {-10, 0}}},
                       {{{{0, 0}, {-1, -1}, {-2, -5}, {-2, 0}},
                         {{0, -10}, {-2, -10}, {-2, -5}, {-1, -9}},
                         {{-2, -5}, {-2, -10}, {-10, -10}, {-10, 0}, {-2, 0}}}}},

        //! \test Test decomposition with t single-point concavities on closing side of outer loop
        PolygonAndDcel{{{{0, 0}, {0, -2}, {-2, -3}, {0, -4}, {0, -6}, {-2, -7}, {0, -8}, {0, -10}, {-10, -10}, {-10, 0}}},
                       {{{{0, 0}, {0, -2}, {-2, -3}, {-2, 0}},
                         {{-2, -3}, {0, -4}, {0, -6}, {-2, -7}},
                         {{0, -8}, {0, -10}, {-2, -10}, {-2, -7}},
                         {{-2, -3}, {-2, -7}, {-2, -10}, {-10, -10}, {-10, 0}, {-2, 0}}}}},

        //! \test Test decomposition with t multi-point concavities on closing side of outer loop
        PolygonAndDcel{{{{0, 0},
                         {0, -2},
                         {-1, -2.25},
                         {-2, -3},
                         {-1, -3.75},
                         {0, -4},
                         {0, -6},
                         {-1, -6.25},
                         {-2, -7},
                         {-1, -7.75},
                         {0, -8},
                         {0, -10},
                         {-10, -10},
                         {-10, 0}}},
                       {{{{0, 0}, {0, -2}, {-1, -2.25}, {-2, -3}, {-2, 0}},
                         {{-2, -3}, {-1, -3.75}, {0, -4}, {0, -6}, {-1, -6.25}, {-2, -7}},
                         {{0, -8}, {0, -10}, {-2, -10}, {-2, -7}, {-1, -7.75}},
                         {{-2, -3}, {-2, -7}, {-2, -10}, {-10, -10}, {-10, 0}, {-2, 0}}}}},

        //! \test Test decomposition with t single-point concavities as the closing side of outer loop
        PolygonAndDcel{{{{0, 0}, {-2, -3}, {0, -5}, {-2, -7}, {0, -10}, {-10, -10}, {-10, 0}}},
                       {{{{0, 0}, {-2, -3}, {-2, 0}},
                         {{-2, -3}, {0, -5}, {-2, -7}},
                         {{0, -10}, {-2, -10}, {-2, -7}},
                         {{-2, -3}, {-2, -7}, {-2, -10}, {-10, -10}, {-10, 0}, {-2, 0}}}}},

        //! \test Test decomposition with t multi-point concavities as the closing side of outer loop
        PolygonAndDcel{
            {{{0, 0}, {-1, -1}, {-2, -3}, {-1, -3.75}, {0, -5}, {-1, -5.25}, {-2, -7}, {-1, -9}, {0, -10}, {-10, -10}, {-10, 0}}},
            {{{{0, 0}, {-1, -1}, {-2, -3}, {-2, 0}},
              {{-2, -3}, {-1, -3.75}, {0, -5}, {-1, -5.25}, {-2, -7}},
              {{0, -10}, {-2, -10}, {-2, -7}, {-1, -9}},
              {{-2, -3}, {-2, -7}, {-2, -10}, {-10, -10}, {-10, 0}, {-2, 0}}}}}

        ));

// Tests for when there are weird concavities on the right side of the inner loop
INSTANTIATE_TEST_SUITE_P(
    CCPPTests_Concavities_Inner_Right, ModifiedTrapezoidalPolygonDecomposition,
    Values(
        //! \test Test decomposition with o single-point concavity on closing side of inner loop
        PolygonAndDcel{{{{0, 0}, {0, -10}, {-10, -10}, {-10, 0}}, {{{-2, -2}, {-2, -4}, {-3, -5}, {-2, -6}, {-2, -8}, {-8, -8}, {-8, -2}}}},
                       {{{{0, 0}, {0, -10}, {-2, -10}, {-2, -8}, {-2, -6}, {-2, -4}, {-2, -2}, {-2, 0}},
                         {{-2, 0}, {-2, -2}, {-8, -2}, {-8, 0}},
                         {{-2, -4}, {-2, -6}, {-3, -5}},
                         {{-2, -8}, {-2, -10}, {-8, -10}, {-8, -8}},
                         {{-8, -10}, {-10, -10}, {-10, 0}, {-8, 0}, {-8, -2}, {-8, -8}}}}},

        //! \test Test decomposition with o multi-point concavity on closing side of inner loop
        PolygonAndDcel{{{{0, 0}, {0, -10}, {-10, -10}, {-10, 0}},
                        {{{-2, -2}, {-2, -4}, {-2.5, -4.25}, {-3, -5}, {-2.5, -5.75}, {-2, -6}, {-2, -8}, {-8, -8}, {-8, -2}}}},
                       {{{{0, 0}, {0, -10}, {-2, -10}, {-2, -8}, {-2, -6}, {-2, -4}, {-2, -2}, {-2, 0}},
                         {{-2, 0}, {-2, -2}, {-8, -2}, {-8, 0}},
                         {{-2, -4}, {-2, -6}, {-2.5, -5.75}, {-3, -5}, {-2.5, -4.25}},
                         {{-2, -8}, {-2, -10}, {-8, -10}, {-8, -8}},
                         {{-8, -10}, {-10, -10}, {-10, 0}, {-8, 0}, {-8, -2}, {-8, -8}}}}},

        //! \test Test decomposition with o single-point concavity as the closing side of inner loop
        PolygonAndDcel{{{{0, 0}, {0, -10}, {-10, -10}, {-10, 0}}, {{{-2, -2}, {-3, -5}, {-2, -8}, {-8, -8}, {-8, -2}}}},
                       {{{{0, 0}, {0, -10}, {-2, -10}, {-2, -8}, {-2, -2}, {-2, 0}},
                         {{-2, 0}, {-2, -2}, {-8, -2}, {-8, 0}},
                         {{-2, -2}, {-2, -8}, {-3, -5}},
                         {{-2, -8}, {-2, -10}, {-8, -10}, {-8, -8}},
                         {{-8, -10}, {-10, -10}, {-10, 0}, {-8, 0}, {-8, -2}, {-8, -8}}}}},

        //! \test Test decomposition with o multi-point concavity as the closing side of inner loop
        PolygonAndDcel{{{{0, 0}, {0, -10}, {-10, -10}, {-10, 0}}, {{{-2, -2}, {-3, -3}, {-4, -5}, {-3, -7}, {-2, -8}, {-8, -8}, {-8, -2}}}},
                       {{{{0, 0}, {0, -10}, {-2, -10}, {-2, -8}, {-2, -2}, {-2, 0}},
                         {{-2, 0}, {-2, -2}, {-8, -2}, {-8, 0}},
                         {{-2, -2}, {-2, -8}, {-3, -7}, {-4, -5}, {-3, -3}},
                         {{-2, -8}, {-2, -10}, {-8, -10}, {-8, -8}},
                         {{-8, -10}, {-10, -10}, {-10, 0}, {-8, 0}, {-8, -2}, {-8, -8}}}}},

        //! \test Test decomposition with t single-point concavities on closing side of inner loop
        PolygonAndDcel{{{{0, 0}, {0, -10}, {-10, -10}, {-10, 0}},
                        {{{-2, -2}, {-2, -3}, {-3, -3.5}, {-2, -4}, {-2, -6}, {-3, -6.5}, {-2, -7}, {-2, -8}, {-8, -8}, {-8, -2}}}},
                       {{{{0, 0}, {0, -10}, {-2, -10}, {-2, -8}, {-2, -7}, {-2, -6}, {-2, -4}, {-2, -3}, {-2, -2}, {-2, 0}},
                         {{-2, 0}, {-2, -2}, {-8, -2}, {-8, 0}},
                         {{-2, -3}, {-2, -4}, {-3, -3.5}},
                         {{-2, -6}, {-2, -7}, {-3, -6.5}},
                         {{-2, -8}, {-2, -10}, {-8, -10}, {-8, -8}},
                         {{-8, -10}, {-10, -10}, {-10, 0}, {-8, 0}, {-8, -2}, {-8, -8}}}}},

        //! \test Test decomposition with t multi-point concavities on closing side of inner loop
        PolygonAndDcel{{{{0, 0}, {0, -10}, {-10, -10}, {-10, 0}},
                        {{{-2, -2},
                          {-2, -3},
                          {-3.75, -3.25},
                          {-4, -3.5},
                          {-3.75, -3.75},
                          {-2, -4},
                          {-2, -6},
                          {-3.75, -6.25},
                          {-4, -6.5},
                          {-3.75, -6.75},
                          {-2, -7},
                          {-2, -8},
                          {-8, -8},
                          {-8, -2}}}},
                       {{{{0, 0}, {0, -10}, {-2, -10}, {-2, -8}, {-2, -7}, {-2, -6}, {-2, -4}, {-2, -3}, {-2, -2}, {-2, 0}},
                         {{-2, 0}, {-2, -2}, {-8, -2}, {-8, 0}},
                         {{-2, -3}, {-2, -4}, {-3.75, -3.75}, {-4, -3.5}, {-3.75, -3.25}},
                         {{-2, -6}, {-2, -7}, {-3.75, -6.75}, {-4, -6.5}, {-3.75, -6.25}},
                         {{-2, -8}, {-2, -10}, {-8, -10}, {-8, -8}},
                         {{-8, -10}, {-10, -10}, {-10, 0}, {-8, 0}, {-8, -2}, {-8, -8}}}}},

        //! \test Test decomposition with t single-point concavities as the closing side of inner loop
        PolygonAndDcel{{{{0, 0}, {0, -10}, {-10, -10}, {-10, 0}}, {{{-2, -2}, {-3, -4}, {-2, -5}, {-3, -6}, {-2, -8}, {-8, -8}, {-8, -2}}}},
                       {{{{0, 0}, {0, -10}, {-2, -10}, {-2, -8}, {-2, -5}, {-2, -2}, {-2, 0}},
                         {{-2, 0}, {-2, -2}, {-8, -2}, {-8, 0}},
                         {{-2, -2}, {-2, -5}, {-3, -4}},
                         {{-2, -5}, {-2, -8}, {-3, -6}},
                         {{-2, -8}, {-2, -10}, {-8, -10}, {-8, -8}},
                         {{-8, -10}, {-10, -10}, {-10, 0}, {-8, 0}, {-8, -2}, {-8, -8}}}}},

        //! \test Test decomposition with t multi-point concavities as the closing side of inner loop
        PolygonAndDcel{
            {{{0, 0}, {0, -10}, {-10, -10}, {-10, 0}},
             {{{-2, -2}, {-3, -2.5}, {-4, -4}, {-3, -4.75}, {-2, -5}, {-3, -5.25}, {-4, -6}, {-3, -7.5}, {-2, -8}, {-8, -8}, {-8, -2}}}},
            {{{{0, 0}, {0, -10}, {-2, -10}, {-2, -8}, {-2, -5}, {-2, -2}, {-2, 0}},
              {{-2, 0}, {-2, -2}, {-8, -2}, {-8, 0}},
              {{-2, -2}, {-2, -5}, {-3, -4.75}, {-4, -4}, {-3, -2.5}},
              {{-2, -5}, {-2, -8}, {-3, -7.5}, {-4, -6}, {-3, -5.25}},
              {{-2, -8}, {-2, -10}, {-8, -10}, {-8, -8}},
              {{-8, -10}, {-10, -10}, {-10, 0}, {-8, 0}, {-8, -2}, {-8, -8}}}}}

        ));

// Specific test cases from things that broke during usage
INSTANTIATE_TEST_SUITE_P(CCPPTests_Real_Edge_Cases, ModifiedTrapezoidalPolygonDecomposition,

                         Values(
                             //! \test Test decomposition with a pike outwards on closing side of exterior loop, where the
                             //! segment below it starts to the right of where the spike starts
                             PolygonAndDcel{{{{0, 0}, {0, 10}, {10, 10}, {10, 8}, {12, 5}, {11, 3.5}, {10, 3}, {10, 0}}},
                                            {{{{0, 0}, {0, 10}, {10, 10}, {10, 8}, {12, 5}, {11, 3.5}, {10, 3}, {10, 0}}}}}

                             ));
