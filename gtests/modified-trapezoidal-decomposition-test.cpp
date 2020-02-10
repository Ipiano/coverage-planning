#include "ads/ccpp/polygon-decomposer/modified-trapezoidal.h"

#include <boost/geometry/algorithms/distance.hpp>

#include <gtest/gtest.h>

#include <iterator>
#include <sstream>
#include <unordered_set>

using namespace testing;
using namespace ads::ccpp;
namespace bg = boost::geometry;

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

// TODO: MAke this work regardless of region ordering
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
    polygon_decomposer::ModifiedTrapezoidal decomposer = {0};
    geometry::Polygon2d poly;

    void SetUp() override
    {
        poly = GetParam().first;
        boost::geometry::correct(poly);

        ASSERT_TRUE(boost::geometry::is_valid(poly));
    }
};

TEST_P(ModifiedTrapezoidalPolygonDecomposition, ProducesCorrectDcel)
{
    const auto dcel = decomposer.decomposePolygon(poly);
    ASSERT_TRUE(DcelIsValid(dcel));
    EXPECT_TRUE(DcelMatchesDescription(dcel, GetParam().second));
}

INSTANTIATE_TEST_SUITE_P(CCPPTests, ModifiedTrapezoidalPolygonDecomposition,
                         Values(

                             // A diamond with 4 points
                             PolygonAndDcel({{{0, 0}, {1, 1}, {2, 0}, {1, -1}}}, {{{{0, 0}, {1, 1}, {2, 0}, {1, -1}}}}),

                             // A diamond with extra colinear points splitting each edge
                             PolygonAndDcel({{{0, 0}, {1, 1}, {2, 2}, {3, 1}, {4, 0}, {3, -1}, {2, -2}, {1, -1}}},
                                            {{{{0, 0}, {1, 1}, {2, 2}, {3, 1}, {4, 0}, {3, -1}, {2, -2}, {1, -1}}}}),

                             // A square with 4 points
                             PolygonAndDcel({{{0, 0}, {0, 1}, {1, 1}, {1, 0}}}, {{{{0, 0}, {0, 1}, {1, 1}, {1, 0}}}}),

                             // A square with extra colinear points splitting each edge
                             PolygonAndDcel({{{0, 0}, {0, 1}, {0, 2}, {1, 2}, {2, 2}, {2, 1}, {2, 0}, {1, 0}}},
                                            {{{{0, 0}, {0, 1}, {0, 2}, {1, 2}, {2, 2}, {2, 1}, {2, 0}, {1, 0}}}}),

                             // A square with a diamond hole in the middle
                             PolygonAndDcel({{{0, 0}, {0, 3}, {3, 3}, {3, 0}}, {{{1, 1.5}, {1.5, 1}, {2, 1.5}, {1.5, 2}}}},
                                            {{{{0, 0}, {0, 3}, {1, 3}, {1, 1.5}, {1, 0}},
                                              {{1, 1.5}, {1, 3}, {2, 3}, {2, 1.5}, {1.5, 2}},
                                              {{1, 1.5}, {1.5, 1}, {2, 1.5}, {2, 0}, {1, 0}},
                                              {{2, 1.5}, {2, 3}, {3, 3}, {3, 0}, {2, 0}}}})

                                 ));
