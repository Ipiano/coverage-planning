#include "ads/dcel/dcel.h"

#include <gtest/gtest.h>

using namespace ads::dcel;
using namespace ads;
namespace bg = boost::geometry;

bool equal(const ccpp::geometry::Point2d& p1, const ccpp::geometry::Point2d& p2)
{
    return std::abs(bg::distance(p1, p2)) < 0.001;
}

//! \test Test that dcel::Dcel creates a vertex when a new point is added
TEST(Dcel, AddVertex)
{
    Dcel dcel;

    const auto v = dcel.vertex({10, 10});
    EXPECT_TRUE(equal(v.point(), ccpp::geometry::Point2d(10, 10)));

    const auto isValidResult = dcel.isValid();
    EXPECT_TRUE(isValidResult.first) << isValidResult.second;
}

//! \test Test that dcel::Dcel does not assign an edge to newly-added vertices
TEST(Dcel, NewVerticesHaveNoEdge)
{
    Dcel dcel;

    const auto v = dcel.vertex({10, 10});

    EXPECT_FALSE(v.edge());

    const auto isValidResult = dcel.isValid();
    EXPECT_TRUE(isValidResult.first) << isValidResult.second;
}

//! \test Test that dcel::Dcel will return an existing vertex instead of creating a
//! new one when a point is accessed the second time
TEST(Dcel, GetExistingVertex)
{
    Dcel dcel;

    const auto v1 = dcel.vertex({10, 10});
    const auto v2 = dcel.vertex({10, 10});

    EXPECT_EQ(v1, v2);

    const auto isValidResult = dcel.isValid();
    EXPECT_TRUE(isValidResult.first) << isValidResult.second;
}

//! \test Test that dcel::Dcel can be used to create a region with a single edge
//! between two existing vertices, and that the edge will have appropriate links.
TEST(Dcel, CreateRegion)
{
    Dcel dcel;

    const auto v1 = dcel.vertex({10, 10});
    const auto v2 = dcel.vertex({10, 11});

    const auto e1 = dcel.createRegion(v1, v2);
    const auto e2 = e1.next();

    EXPECT_NE(e1, e2);

    EXPECT_TRUE(e1.region());
    EXPECT_EQ(e1.region(), e2.region());

    EXPECT_EQ(v1, e1.origin());
    EXPECT_EQ(v2, e2.origin());

    EXPECT_EQ(v1.edge(), e1);
    EXPECT_EQ(v2.edge(), e2);

    EXPECT_EQ(e1.next(), e2);
    EXPECT_EQ(e1.previous(), e2);

    EXPECT_EQ(e2.next(), e1);
    EXPECT_EQ(e2.previous(), e1);

    EXPECT_FALSE(e1.twin());
    EXPECT_FALSE(e2.twin());

    const auto isValidResult = dcel.isValid();
    EXPECT_TRUE(isValidResult.first) << isValidResult.second;
}

//! \test Test that dcel::Dcel can split an edge into two adjacent ones in the same region
TEST(Dcel, SplitEdgeOnce)
{
    Dcel dcel;

    const auto v1 = dcel.vertex({10, 10});
    const auto v2 = dcel.vertex({10, 11});

    const auto e1 = dcel.createRegion(v1, v2);
    const auto e3 = e1.next();

    const auto v3 = dcel.vertex({11, 11});
    const auto e2 = dcel.splitEdge(e1, v3);

    EXPECT_TRUE(e1.region());
    EXPECT_EQ(e1.region(), e2.region());

    EXPECT_EQ(e1.origin(), v1);
    EXPECT_EQ(e2.origin(), v3);

    EXPECT_EQ(e1.next(), e2);
    EXPECT_EQ(e2.next(), e3);

    EXPECT_EQ(e3.previous(), e2);
    EXPECT_EQ(e2.previous(), e1);

    const auto isValidResult = dcel.isValid();
    EXPECT_TRUE(isValidResult.first) << isValidResult.second;
}

//! \test Test that dcel::Dcel can split an edge into two adjacent ones in the same region twice
TEST(Dcel, SplitEdgeTwice)
{
    Dcel dcel;

    const auto v1 = dcel.vertex({10, 10});
    const auto v2 = dcel.vertex({9, 9});

    const auto e1 = dcel.createRegion(v1, v2);
    const auto e2 = e1.next();

    const auto v3 = dcel.vertex({11, 11});
    const auto e3 = dcel.splitEdge(e1, v3);

    const auto v4 = dcel.vertex({10, 11});
    const auto e4 = dcel.splitEdge(e1, v4);

    EXPECT_TRUE(e1.region());
    EXPECT_EQ(e1.region(), e3.region());
    EXPECT_EQ(e1.region(), e4.region());

    EXPECT_EQ(e1.origin(), v1);
    EXPECT_EQ(e3.origin(), v3);
    EXPECT_EQ(e4.origin(), v4);

    EXPECT_EQ(e1.next(), e4);
    EXPECT_EQ(e4.next(), e3);
    EXPECT_EQ(e3.next(), e2);

    EXPECT_EQ(e2.previous(), e3);
    EXPECT_EQ(e3.previous(), e4);
    EXPECT_EQ(e4.previous(), e1);

    const auto isValidResult = dcel.isValid();
    EXPECT_TRUE(isValidResult.first) << isValidResult.second;
}

//! \test Test that dcel::Dcel can split a region into two by creating an edge between two non-adjacent edges
TEST(Dcel, SplitRegionInMiddle)
{
    Dcel dcel;

    const auto v1 = dcel.vertex({10, 10});
    const auto v2 = dcel.vertex({0, 0});
    const auto v3 = dcel.vertex({10, 0});
    const auto v4 = dcel.vertex({0, 10});

    const auto e1 = dcel.createRegion(v2, v4);
    const auto e2 = e1.next();
    const auto e3 = dcel.splitEdge(e2, v1);
    const auto e4 = dcel.splitEdge(e3, v3);

    const auto e5 = dcel.splitRegion(e2, e1);
    const auto e6 = e5.twin();

    EXPECT_TRUE(e6);
    EXPECT_EQ(e6.twin(), e5);

    EXPECT_EQ(e5.origin(), e3.origin());
    EXPECT_EQ(e6.origin(), e1.origin());

    EXPECT_EQ(e1.next(), e2);
    EXPECT_EQ(e2.next(), e5);
    EXPECT_EQ(e5.next(), e1);

    EXPECT_EQ(e3.next(), e4);
    EXPECT_EQ(e4.next(), e6);
    EXPECT_EQ(e6.next(), e3);

    EXPECT_EQ(e1.region(), e2.region());
    EXPECT_EQ(e2.region(), e5.region());

    EXPECT_NE(e5.region(), e6.region());

    EXPECT_EQ(e3.region(), e4.region());
    EXPECT_EQ(e4.region(), e6.region());

    const auto isValidResult = dcel.isValid();
    EXPECT_TRUE(isValidResult.first) << isValidResult.second;
}

//! \test Test that dcel::Dcel can split an edge that has a twin and the twin will be split appropriately also
TEST(Dcel, SplitEdgeWithTwin)
{
    Dcel dcel;

    const auto v1 = dcel.vertex({10, 10});
    const auto v2 = dcel.vertex({0, 0});
    const auto v3 = dcel.vertex({10, 0});
    const auto v4 = dcel.vertex({0, 10});
    const auto v5 = dcel.vertex({5, 5});

    const auto e1 = dcel.createRegion(v2, v4);
    const auto e2 = e1.next();
    const auto e3 = dcel.splitEdge(e2, v1);
    const auto e4 = dcel.splitEdge(e3, v3);

    const auto e5 = dcel.splitRegion(e2, e1);
    const auto e6 = e5.twin();

    const auto e7 = dcel.splitEdge(e5, v5);
    const auto e8 = e6.next();

    EXPECT_TRUE(e5.twin());
    EXPECT_TRUE(e6.twin());
    EXPECT_TRUE(e7.twin());
    EXPECT_TRUE(e8.twin());

    EXPECT_EQ(e5.twin(), e8);
    EXPECT_EQ(e8.twin(), e5);

    EXPECT_EQ(e6.twin(), e7);
    EXPECT_EQ(e7.twin(), e6);

    EXPECT_EQ(e4.next(), e6);
    EXPECT_EQ(e6.next(), e8);
    EXPECT_EQ(e8.next(), e3);

    EXPECT_EQ(e2.next(), e5);
    EXPECT_EQ(e5.next(), e7);
    EXPECT_EQ(e7.next(), e1);

    EXPECT_EQ(e4.region(), e6.region());
    EXPECT_EQ(e6.region(), e8.region());
    EXPECT_EQ(e8.region(), e3.region());

    EXPECT_EQ(e2.region(), e5.region());
    EXPECT_EQ(e5.region(), e7.region());
    EXPECT_EQ(e7.region(), e1.region());

    const auto isValidResult = dcel.isValid();
    EXPECT_TRUE(isValidResult.first) << isValidResult.second;
}

//! \test Test that dcel::Dcel can split an edge that has a twin and the twin will be split appropriately also.
//! This differs from SplitEdgeWithTwin in that it passes an edge that was created indirectly as a twin to splitEdge instead
//! of an edge that was created directly
TEST(Dcel, SplitEdgeWithTwin2)
{
    Dcel dcel;

    const auto v1 = dcel.vertex({10, 10});
    const auto v2 = dcel.vertex({0, 0});
    const auto v3 = dcel.vertex({10, 0});
    const auto v4 = dcel.vertex({0, 10});
    const auto v5 = dcel.vertex({5, 5});

    const auto e1 = dcel.createRegion(v2, v4);
    const auto e2 = e1.next();
    const auto e3 = dcel.splitEdge(e2, v1);
    const auto e4 = dcel.splitEdge(e3, v3);

    const auto e5 = dcel.splitRegion(e2, e1);
    const auto e6 = e5.twin();

    const auto e8 = dcel.splitEdge(e6, v5);
    const auto e7 = e1.previous();

    EXPECT_TRUE(e5.twin());
    EXPECT_TRUE(e6.twin());
    EXPECT_TRUE(e7.twin());
    EXPECT_TRUE(e8.twin());

    EXPECT_EQ(e5.twin(), e8);
    EXPECT_EQ(e8.twin(), e5);

    EXPECT_EQ(e6.twin(), e7);
    EXPECT_EQ(e7.twin(), e6);

    EXPECT_EQ(e4.next(), e6);
    EXPECT_EQ(e6.next(), e8);
    EXPECT_EQ(e8.next(), e3);

    EXPECT_EQ(e2.next(), e5);
    EXPECT_EQ(e5.next(), e7);
    EXPECT_EQ(e7.next(), e1);

    EXPECT_EQ(e4.region(), e6.region());
    EXPECT_EQ(e6.region(), e8.region());
    EXPECT_EQ(e8.region(), e3.region());

    EXPECT_EQ(e2.region(), e5.region());
    EXPECT_EQ(e5.region(), e7.region());
    EXPECT_EQ(e7.region(), e1.region());

    const auto isValidResult = dcel.isValid();
    EXPECT_TRUE(isValidResult.first) << isValidResult.second;
}

//! \test Test that dcel::Dcel can merge two regions and remove the unused edges
TEST(Dcel, MergeRegions)
{
    Dcel dcel;

    const auto v1 = dcel.vertex({10, 10});
    const auto v2 = dcel.vertex({0, 0});
    const auto v3 = dcel.vertex({10, 0});
    const auto v4 = dcel.vertex({0, 10});
    const auto v5 = dcel.vertex({10, -10});
    const auto v6 = dcel.vertex({0, -10});

    const auto e1 = dcel.createRegion(v2, v4);
    const auto e2 = e1.next();
    const auto e3 = dcel.splitEdge(e2, v1);
    const auto e4 = dcel.splitEdge(e3, v3);

    const auto e5 = dcel.createRegion(v2, v3);
    const auto e6 = e5.next();
    const auto e7 = dcel.splitEdge(e6, v5);
    const auto e8 = dcel.splitEdge(e7, v6);

    dcel.mergeRegions(e8, e1, e3, e6);

    EXPECT_FALSE(e4);
    EXPECT_FALSE(e5);

    EXPECT_EQ(e8.next(), e1);
    EXPECT_EQ(e3.next(), e6);

    const auto isValidResult = dcel.isValid();
    EXPECT_TRUE(isValidResult.first) << isValidResult.second;
}
