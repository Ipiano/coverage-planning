#include "dcel.h"

#include <boost/geometry/index/parameters.hpp>
#include <boost/geometry/geometries/box.hpp>

namespace bg  = boost::geometry;
namespace bgi = bg::index;

namespace ads
{
namespace ccpp
{

typedef DoublyConnectedEdgeList DCEL;

DCEL::DoublyConnectedEdgeList(const geometry::Polygon2d& initialShape, const double epsilon) : m_epsilon(std::abs(epsilon))
{
    // Should have 4 points because polygon rings are closed
    // so first and last points match
    if (initialShape.outer().size() < 4)
        throw std::runtime_error("polygon outer ring has too few points");

    for (const auto& ring : initialShape.inners())
        if (ring.size() < 4)
            throw std::runtime_error("polygon inner ring has too few points");

    m_faces.emplace_back(new dcel::face_t);
    m_faces.emplace_back(new dcel::face_t);

    auto outerFace = m_faces[0].get();
    auto innerFace = m_faces[1].get();

    addLoop(initialShape.outer(), innerFace, outerFace);
    for (const auto& ring : initialShape.inners())
    {
        m_faces.emplace_back(new dcel::face_t);
        auto innerInnerFace = m_faces.back().get();
        addLoop(ring, innerFace, innerInnerFace);
    }
}

DCEL::DoublyConnectedEdgeList(DCEL&& orig)
    : m_vertices(std::move(orig.m_vertices)), m_faces(std::move(orig.m_faces)), m_edges(std::move(orig.m_edges))
{
    orig.m_vertices.clear();
    orig.m_faces.clear();
    orig.m_edges.clear();
}

const dcel::const_face_t* DCEL::insideFace() const
{
    return reinterpret_cast<dcel::const_face_t*>(m_faces[1].get());
}

std::vector<const dcel::const_half_edge_t*> DCEL::edges(const dcel::const_face_t* face) const
{
    std::vector<const dcel::const_half_edge_t*> result;
    result.reserve(m_edges.size());

    for (const auto& edge : m_edges)
    {
        if (reinterpret_cast<const dcel::const_face_t*>(edge->face) == face)
            result.push_back(reinterpret_cast<const dcel::const_half_edge_t*>(edge.get()));
    }
    return result;
}

void DCEL::addLoop(const geometry::Ring2d& ring, dcel::face_t* rightFace, dcel::face_t* leftFace)
{
    dcel::half_edge_t* previousRight = nullptr;
    dcel::half_edge_t* nextLeft      = nullptr;

    dcel::half_edge_t* firstRight = nullptr;
    dcel::half_edge_t* firstLeft  = nullptr;

    m_edges.reserve(m_edges.size() + ring.size() * 2);

    // Assumption: This travels the ring in order
    bg::for_each_segment(ring, [&](const bg::model::referring_segment<const geometry::Point2d>& segment) {
        auto pt1 = findOrCreate(segment.first);
        auto pt2 = findOrCreate(segment.second);

        auto edge1 = new dcel::half_edge_t;
        auto edge2 = new dcel::half_edge_t;

        edge1->face   = rightFace;
        edge1->prev   = previousRight;
        edge1->twin   = edge2;
        edge1->origin = pt1;

        if (previousRight)
            previousRight->next = edge1;

        if (!pt1->edge)
            pt1->edge = edge1;

        edge2->face   = leftFace;
        edge2->next   = nextLeft;
        edge2->twin   = edge1;
        edge2->origin = pt2;

        if (nextLeft)
            nextLeft->prev = edge2;

        if (!pt2->edge)
            pt2->edge = edge2;

        if (!firstRight)
        {
            firstRight = edge1;
            firstLeft  = edge2;
        }

        previousRight = edge1;
        nextLeft      = edge2;

        m_edges.emplace_back(edge1);
        m_edges.emplace_back(edge2);
    });

    firstLeft->next = nextLeft;
    nextLeft->prev  = firstLeft;

    firstRight->prev    = previousRight;
    previousRight->next = firstRight;

    if (!leftFace->edge)
        leftFace->edge = firstLeft;

    if (!rightFace->edge)
        rightFace->edge = firstRight;
}

dcel::vertex_t* DCEL::findOrCreate(const geometry::Point2d& point)
{
    auto maybePoint = find(point);
    if (!maybePoint)
    {
        maybePoint           = new dcel::vertex_t;
        maybePoint->location = point;
        m_vertices.insert(std::shared_ptr<dcel::vertex_t>(maybePoint));
    }
    return maybePoint;
}

dcel::vertex_t* DCEL::find(const geometry::Point2d& target) const
{
    const auto cornerDelta = bg::make<geometry::Point2d>(m_epsilon, m_epsilon);

    auto minCorner = target, maxCorner = target;
    bg::subtract_point(minCorner, cornerDelta);
    bg::add_point(maxCorner, cornerDelta);

    bg::model::box<geometry::Point2d> searchBox(minCorner, maxCorner);

    std::array<std::shared_ptr<dcel::vertex_t>, 1> result = {{{}}};
    m_vertices.query(bgi::within(searchBox) && bgi::nearest(target, 1), result.begin());

    return result[0].get();
}
}
}
