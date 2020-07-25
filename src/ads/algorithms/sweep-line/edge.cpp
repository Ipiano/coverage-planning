#include "edge.h"

#include "ads/assertion.h"

#include <boost/geometry/algorithms/distance.hpp>

namespace bg = boost::geometry;

namespace ads
{
namespace algorithms
{
namespace sweep_line
{

Edge::Edge(const Point2d& p1, const Point2d& p2) : m_p1(p1), m_p2(p2)
{
    // Sort the points of the edge
    if (pointLessThan(p2, p1))
        std::swap(m_p1, m_p2);

    // Pre-calculate other values that the edge provides
    m_isVertical = haveSameXCoord(m_p1, m_p2);

    const double mag = bg::distance(m_p1, m_p2);

    Point2d unit = m_p2;
    bg::subtract_point(unit, m_p1);
    bg::divide_value(unit, mag);

    m_angleSortValue = bg::dot_product(unit, Point2d{0, 1});
}

const Point2d& Edge::firstPoint() const
{
    return m_p1;
}

const Point2d& Edge::secondPoint() const
{
    return m_p2;
}

bool Edge::isVertical() const
{
    return m_isVertical;
}

bool Edge::hasPoint(const Point2d& point) const
{
    return equal(point, firstPoint()) || equal(point, secondPoint());
}

bool Edge::operator<(const Edge& other) const
{
    if (equal(firstPoint(), other.firstPoint()))
        return angleLessThan(other);

    return pointLessThan(firstPoint(), other.firstPoint());
}

double Edge::unitDotHorizontal() const
{
    return m_angleSortValue;
}

PolygonEdge::PolygonEdge(const Point2d& p1, const Point2d& p2) : Edge(p1, p2), m_nextEdge(nullptr), m_previousEdge(nullptr)
{
}

// All of these calls assert that the edges were set by a call to
// setAdjacentEdges because that should always get called during
// sweep line setup

bool PolygonEdge::isClockwise()
{
    ASSERT(m_nextEdge);
    return m_isClockwise;
}

PolygonEdge* PolygonEdge::previousEdge() const
{
    ASSERT(m_nextEdge);
    return m_previousEdge;
}

PolygonEdge* PolygonEdge::nextEdge() const
{
    ASSERT(m_nextEdge);
    return m_nextEdge;
}

PolygonEdge* PolygonEdge::clockwiseEdge() const
{
    ASSERT(m_nextEdge);
    return m_nextIsClockwise ? m_nextEdge : m_previousEdge;
}

PolygonEdge* PolygonEdge::counterclockwiseEdge() const
{
    ASSERT(m_nextEdge);
    return m_nextIsClockwise ? m_previousEdge : m_nextEdge;
}

PolygonEdge* PolygonEdge::firstEdge() const
{
    ASSERT(m_nextEdge);
    return m_nextIsFirst ? m_nextEdge : m_previousEdge;
}

PolygonEdge* PolygonEdge::secondEdge() const
{
    ASSERT(m_nextEdge);
    return m_nextIsFirst ? m_previousEdge : m_nextEdge;
}

bool PolygonEdge::isVerticalZigZag() const
{
    return m_isVZigZag;
}

// TODO: Not guaranteed to sort correctly when
// two edges on separate loops have the same first
// point and same angle
bool PolygonEdge::operator<(const PolygonEdge& other)
{
    if (equal(firstPoint(), other.firstPoint()))
    {
        // Always process vertical zig-zag edges before
        // edge connected to their bottom
        // point; this makes sure that we're traversing
        // around the loops in order and not skipping over
        // a random vertical edge that's not a start or end of loop
        if (isVerticalZigZag())
            return true;

        if (other.isVerticalZigZag())
            return false;
    }

    return Edge::operator<(other);
}

void PolygonEdge::setAdjacentEdges(PolygonEdge* previous, PolygonEdge* next, bool isClockwise)
{
    // Should only be called once
    ASSERT(!m_nextEdge);

    m_isClockwise  = isClockwise;
    m_previousEdge = previous;
    m_nextEdge     = next;

    m_nextIsClockwise = m_isClockwise;
    m_nextIsFirst     = nextEdge()->hasPoint(firstPoint());

    m_isVZigZag = isVertical() && ((equal(secondEdge()->secondPoint(), secondPoint()) && equal(firstEdge()->firstPoint(), firstPoint())) ||
                                   (equal(secondEdge()->firstPoint(), secondPoint()) && equal(firstEdge()->secondPoint(), firstPoint())));
}
}
}
}
