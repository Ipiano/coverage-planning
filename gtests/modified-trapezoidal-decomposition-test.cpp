#include "ads/ccpp/polygon-decomposer/modified-trapezoidal.h"

#include <boost/geometry/algorithms/distance.hpp>
#include <boost/geometry/multi/geometries/multi_point.hpp>
#include <boost/geometry/algorithms/intersection.hpp>

#include <gtest/gtest.h>

#include <iterator>
#include <sstream>
#include <unordered_set>

using namespace testing;
using namespace ads::ccpp;
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
AssertionResult DcelIsValid(const DoublyConnectedEdgeList& dcel)
{
    std::unordered_set<const dcel::region_t*> regionSet;
    std::unordered_set<const dcel::half_edge_t*> edgeSet;
    std::unordered_set<const dcel::vertex_t*> vertexSet;

    for (size_t i = 0; i < dcel.regions.size(); i++)
    {
        const auto& region = dcel.regions[i];

        if (!region)
            return AssertionFailure() << "region " << i << " is null";

        if (!region->edge)
            return AssertionFailure() << "retgion " << i << " has null edge";

        regionSet.insert(region.get());
    }

    for (size_t i = 0; i < dcel.edges.size(); i++)
    {
        const auto& edge = dcel.edges[i];

        if (!edge)
            return AssertionFailure() << "edge " << i << " is null";

        if (!edge->next || !edge->prev)
            return AssertionFailure() << "edge " << i << " has null neighbor";

        if (!edge->origin)
            return AssertionFailure() << "edge " << i << " has null origin";

        if (!edge->region)
            return AssertionFailure() << "edge " << i << " has null region";

        edgeSet.insert(edge.get());
    }

    for (size_t i = 0; i < dcel.vertices.size(); i++)
    {
        const auto& vertex = dcel.vertices[i];

        if (!vertex)
            return AssertionFailure() << "vertex " << i << " is null";

        if (!vertex->edge)
            return AssertionFailure() << "vertex " << i << " has null edge";

        if (edgeSet.count(vertex->edge) == 0)
            return AssertionFailure() << "vertex " << i << " points to unknown edge";

        vertexSet.insert(vertex.get());
    }

    for (size_t i = 0; i < dcel.edges.size(); i++)
    {
        const auto& edge = dcel.edges[i];

        if (edgeSet.count(edge->next) == 0 || edgeSet.count(edge->prev) == 0)
            return AssertionFailure() << "edge " << i << " has unknown neighbor";

        if (edge->twin)
        {
            if (edgeSet.count(edge->twin) == 0)
                return AssertionFailure() << "edge " << i << " has unknown twin";

            if (edge->twin->twin != edge.get())
                return AssertionFailure() << "edge " << i << " twin does not have edge " << i << " as twin";
        }

        if (vertexSet.count(edge->origin) == 0)
            return AssertionFailure() << "edge " << i << " has unknown origin";

        if (regionSet.count(edge->region) == 0)
            return AssertionFailure() << "edge " << i << " has unknown region";

        if (edge->region != edge->next->region || edge->region != edge->prev->region)
            return AssertionFailure() << "edge " << i << " does not share region with neighbor";

        if (edge->next->prev != edge.get() || edge->prev->next != edge.get())
            return AssertionFailure() << "edge " << i << " is not properly linked to neighbors";

        if (edge->next == edge.get() || edge->prev == edge.get())
            return AssertionFailure() << "edge " << i << " is neighbor to itself";

        if (edge->next == edge->prev)
            return AssertionFailure() << "edge " << i << " has same next and previous";
    }

    for (size_t i = 0; i < dcel.regions.size(); i++)
    {
        const auto& region = dcel.regions[i];

        if (edgeSet.count(region->edge) == 0)
            return AssertionFailure() << "region " << i << " points to unknown edge";

        if (region->edge->region != region.get())
            return AssertionFailure() << "region " << i << " points to edge that does not point back";
    }

    return AssertionSuccess();
}

// GTest checker that a DCEL has a specific structure, meaning
// that it has the expected regions and that each region has the expected
// set of points in the correct order
//
// TODO: Currently the regions must be in the same order in both pieces; that should
// not be necessary
AssertionResult DcelMatchesDescription(const DoublyConnectedEdgeList& dcel, const DcelDescription& description)
{
    if (dcel.regions.size() != description.regions.size())
        return AssertionFailure() << "dcel contains wrong number of regions :: got" << dcel.regions.size() << " expected "
                                  << description.regions.size();

    auto resultIt   = dcel.regions.begin();
    auto expectedIt = description.regions.begin();

    for (; resultIt != dcel.regions.end(); resultIt++, expectedIt++)
    {
        const auto regionIndex     = std::distance(description.regions.begin(), expectedIt);
        const auto& resultRegion   = *(resultIt->get());
        const auto& expectedRegion = *expectedIt;

        if (expectedRegion.empty())
            return AssertionFailure() << "dcel description contains empty region (" << regionIndex << ")";

        bool foundStart         = false;
        dcel::half_edge_t* edge = resultRegion.edge;

        do
        {
            if (same(edge->origin->location, expectedRegion.front()))
            {
                foundStart = true;
                break;
            }
        } while ((edge = edge->next) != resultRegion.edge);

        if (!foundStart)
            return AssertionFailure() << "expected region (" << regionIndex << ") start point" << to_string(expectedRegion.front())
                                      << "not found";

        const dcel::half_edge_t* const firstEdge = edge;
        unsigned int matched                     = 0;
        auto expectedPointIt                     = expectedRegion.begin();

        do
        {
            if (!same(edge->origin->location, *expectedPointIt))
            {
                return AssertionFailure() << "point " << std::distance(expectedRegion.begin(), expectedPointIt) << " of region "
                                          << regionIndex << " does not match :: "
                                          << "got " << to_string(edge->origin->location) << " expected " << to_string(*expectedPointIt);
            }
            matched++;
        } while ((edge = edge->next) != firstEdge && ++expectedPointIt != expectedRegion.end());

        if (matched != expectedRegion.size())
            return AssertionFailure() << "region " << regionIndex << " does not contain enough points :: got " << matched << " expected "
                                      << expectedRegion.size();

        if (edge != firstEdge)
        {
            do
            {
                matched++;
            } while ((edge = edge->next) != firstEdge);
            return AssertionFailure() << "region " << regionIndex << " contains too many points :: got " << matched << " expected "
                                      << expectedRegion.size();
        }
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
    const geometry::Segment2d verticalLine {{target.x(), 100}, {target.x(), -100}};

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
        int j = 0;
        for (auto inner : shapes)
        {
            bg::correct(inner);

            cases.push_back(exteriorLoopCase(outer));
            cases.push_back(exteriorLoopCase(addColinearities(outer)));
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

INSTANTIATE_TEST_SUITE_P(DISABLED_CCPPTests_Basic, ModifiedTrapezoidalPolygonDecomposition,
                         ValuesIn(permuteCompositions({SQUARE, DIAMOND, HEXAGON_VERTICAL, HEXAGON_HORIZONTAL, OCTOGON, DODECAHEDRON})));

// Tests for when there are weird concavities in the shapes
INSTANTIATE_TEST_SUITE_P(
    CCPPTests_Concavities, ModifiedTrapezoidalPolygonDecomposition,
    Values(

        // One simple concavity on opening side of inner loop
        PolygonAndDcel {{{{0, 0}, {0, 10}, {10, 10}, {10, 0}}, {{{2, 2}, {2, 4}, {3, 5}, {2, 6}, {2, 8}, {8, 8}, {8, 2}}}},
                        {{{{0, 0}, {0, 10}, {2, 10}, {2, 8}, {2, 6}, {2, 4}, {2, 2}, {2, 0}},
                          {{2, 0}, {2, 2}, {8, 2}, {8, 0}},
                          {{2, 4}, {2, 6}, {3, 5}},
                          {{2, 8}, {2, 10}, {8, 10}, {8, 8}},
                          {{8, 10}, {10, 10}, {10, 0}, {8, 0}, {8, 2}, {8, 8}}}}},

        // One simple concavity as the opening side of inner loop
        PolygonAndDcel {{{{0, 0}, {0, 10}, {10, 10}, {10, 0}}, {{{2, 2}, {3, 5}, {2, 8}, {8, 8}, {8, 2}}}},
                        {{{{0, 0}, {0, 10}, {2, 10}, {2, 8}, {2, 2}, {2, 0}},
                          {{2, 0}, {2, 2}, {8, 2}, {8, 0}},
                          {{2, 2}, {2, 8}, {3, 5}},
                          {{2, 8}, {2, 10}, {8, 10}, {8, 8}},
                          {{8, 10}, {10, 10}, {10, 0}, {8, 0}, {8, 2}, {8, 8}}}}},

        // Two simple concavities on opening side of inner loop
        PolygonAndDcel {
            {{{0, 0}, {0, 10}, {10, 10}, {10, 0}}, {{{2, 2}, {2, 3}, {3, 3.5}, {2, 4}, {2, 6}, {3, 6.5}, {2, 7}, {2, 8}, {8, 8}, {8, 2}}}},
            {{{{0, 0}, {0, 10}, {2, 10}, {2, 8}, {2, 7}, {2, 6}, {2, 4}, {2, 3}, {2, 2}, {2, 0}},
              {{2, 0}, {2, 2}, {8, 2}, {8, 0}},
              {{2, 3}, {2, 4}, {3, 3.5}},
              {{2, 6}, {2, 7}, {3, 6.5}},
              {{2, 8}, {2, 10}, {8, 10}, {8, 8}},
              {{8, 10}, {10, 10}, {10, 0}, {8, 0}, {8, 2}, {8, 8}}}}},

        // Two simple concavities as the opening side of inner loop
        PolygonAndDcel {{{{0, 0}, {0, 10}, {10, 10}, {10, 0}}, {{{2, 2}, {3, 4}, {2, 5}, {3, 6}, {2, 8}, {8, 8}, {8, 2}}}},
                        {{{{0, 0}, {0, 10}, {2, 10}, {2, 8}, {2, 5}, {2, 2}, {2, 0}},
                          {{2, 0}, {2, 2}, {8, 2}, {8, 0}},
                          {{2, 2}, {2, 5}, {3, 4}},
                          {{2, 5}, {2, 8}, {3, 6}},
                          {{2, 8}, {2, 10}, {8, 10}, {8, 8}},
                          {{8, 10}, {10, 10}, {10, 0}, {8, 0}, {8, 2}, {8, 8}}}}}));
