#include "dcel.h"

#include "ads/dcel/exceptions.h"

#include <unordered_set>
#include <sstream>

using namespace std;
namespace bg = boost::geometry;

namespace ads
{
namespace dcel
{

Dcel::VertexData::VertexData(VertexHandle handle, const Point2d& location) : m_handle(handle), m_location(location)
{
}

Dcel::VertexData::~VertexData()
{
    assert(m_edges.size() == 0);
}

VertexHandle Dcel::VertexData::handle() const
{
    return m_handle;
}

const Point2d& Dcel::VertexData::location() const
{
    return m_location;
}

const std::vector<Dcel::HalfEdgeData*>& Dcel::VertexData::edges() const
{
    return m_edges;
}

Dcel::RegionData::RegionData(RegionHandle handle) : m_handle(handle)
{
}

RegionHandle Dcel::RegionData::handle() const
{
    return m_handle;
}

Dcel::HalfEdgeData* Dcel::RegionData::edge() const
{
    return m_edge;
}

void Dcel::RegionData::setEdge(HalfEdgeData* edge)
{
    m_edge = edge;
}

Dcel::HalfEdgeData::HalfEdgeData(HalfEdgeHandle handle, VertexData* origin) : m_handle(handle), m_next(this), m_prev(this), m_origin(origin)
{
    assert(origin != nullptr);
    m_origin->m_edges.push_back(this);
}

Dcel::HalfEdgeData::~HalfEdgeData()
{
    auto& originEdges = m_origin->m_edges;
    originEdges.erase(std::remove(originEdges.begin(), originEdges.end(), this), originEdges.end());

    if (m_twin)
        m_twin->m_twin = nullptr;
}

HalfEdgeHandle Dcel::HalfEdgeData::handle() const
{
    return m_handle;
}

Dcel::HalfEdgeData* Dcel::HalfEdgeData::next() const
{
    return m_next;
}

Dcel::HalfEdgeData* Dcel::HalfEdgeData::prev() const
{
    return m_prev;
}

Dcel::HalfEdgeData* Dcel::HalfEdgeData::twin() const
{
    return m_twin;
}

Dcel::VertexData* Dcel::HalfEdgeData::origin() const
{
    return m_origin;
}

Dcel::RegionData* Dcel::HalfEdgeData::region() const
{
    return m_region;
}

void Dcel::HalfEdgeData::setNext(HalfEdgeData* edge)
{
    assert(edge != nullptr);

    if (edge->m_prev != edge)
        edge->m_prev->m_next = edge->m_prev;

    if (m_next != this)
        m_next->m_prev = m_next;

    m_next       = edge;
    edge->m_prev = this;
}

void Dcel::HalfEdgeData::setPrev(HalfEdgeData* edge)
{
    assert(edge != nullptr);

    if (edge->m_next != edge)
        edge->m_next->m_prev = edge->m_next;

    if (m_prev != this)
        m_prev->m_next = m_prev;

    m_prev       = edge;
    edge->m_next = this;
}

void Dcel::HalfEdgeData::setTwin(HalfEdgeData* edge)
{
    if (m_twin)
        m_twin->m_twin = nullptr;

    m_twin = edge;

    if (m_twin)
    {
        if (m_twin->m_twin)
            edge->m_twin->m_twin = nullptr;

        edge->m_twin = this;
    }
}

void Dcel::HalfEdgeData::setOrigin(VertexData* vertex)
{
    assert(vertex != nullptr);

    auto& originEdges = m_origin->m_edges;
    originEdges.erase(std::remove(originEdges.begin(), originEdges.end(), this), originEdges.end());

    m_origin = vertex;
    m_origin->m_edges.push_back(this);
}

void Dcel::HalfEdgeData::setRegion(RegionData* region)
{
    assert(region != nullptr);
    m_region = region;
}

Dcel::Dcel(const double epsilon)
    : m_nextVertex(1), m_nextRegion(1), m_nextEdge(1), m_epsilon(epsilon), m_epsilonVector(epsilon / 2, epsilon / 2)
{
}

Dcel::~Dcel()
{
    m_edges.clear();
    m_regions.clear();

    // Important that at least m_edges be cleared before vertices
    m_vertices.clear();
    m_points.clear();
}

Dcel::RegionPtr Dcel::makeRegion()
{
    auto ptr = m_regions.emplace(m_nextRegion, new RegionData(m_nextRegion)).first->second;
    m_nextRegion++;

    return ptr;
}

Dcel::HalfEdgePtr Dcel::makeEdge(VertexData* origin)
{
    auto ptr = m_edges.emplace(m_nextEdge, new HalfEdgeData(m_nextEdge, origin)).first->second;
    m_nextEdge++;

    return ptr;
}

Dcel::VertexPtr Dcel::makeVertex(const Point2d& location)
{
    auto ptr = m_vertices.emplace(m_nextVertex, new VertexData(m_nextVertex, location)).first->second;
    m_nextVertex++;

    m_points.insert(ptr);

    return ptr;
}

Dcel::VertexPtr Dcel::getVertex(Vertex v) const
{
    if (this != v.dcel())
        throw InvalidHandleError("vertex belongs to different dcel");

    if (m_vertices.count(v.handle()) == 0)
        throw InvalidHandleError("vertex " + to_string(v.handle().t) + " not found");

    return m_vertices.at(v.handle());
}

bool Dcel::hasVertex(Vertex v) const
{
    return this == v.dcel() && m_vertices.count(v.handle()) != 0;
}

Dcel::RegionPtr Dcel::getRegion(Region r) const
{
    if (this != r.dcel())
        throw InvalidHandleError("region belongs to different dcel");

    if (m_regions.count(r.handle()) == 0)
        throw InvalidHandleError("region " + to_string(r.handle().t) + " not found");

    return m_regions.at(r.handle());
}

bool Dcel::hasRegion(Region r) const
{
    return this == r.dcel() && m_regions.count(r.handle()) != 0;
}

Dcel::HalfEdgePtr Dcel::getEdge(HalfEdge e) const
{
    if (this != e.dcel())
        throw InvalidHandleError("edge belongs to different dcel");

    if (m_edges.count(e.handle()) == 0)
        throw InvalidHandleError("region " + to_string(e.handle().t) + " not found");

    return m_edges.at(e.handle());
}

bool Dcel::hasEdge(HalfEdge e) const
{
    return this == e.dcel() && m_edges.count(e.handle()) != 0;
}

Dcel::VertexPtr Dcel::getPoint(const Point2d& point) const
{
    // Check if a point close enough to be the same has been inserted yet, or
    // insert a new one, in order to have a pointer to assign
    auto minCorner = point, maxCorner = point;
    bg::subtract_point(minCorner, m_epsilonVector);
    bg::add_point(maxCorner, m_epsilonVector);

    const bg::model::box<Point2d> searchBox(minCorner, maxCorner);

    VertexPtr result;
    m_points.query(bg::index::within(searchBox) && bg::index::nearest(point, 1), &result);

    return result;
}

Vertex Dcel::vertex(const Point2d& point) const
{
    auto result = getPoint(point);

    if (!result)
        return Vertex();

    return Vertex(this, result->handle());
}

Vertex Dcel::vertex(const Point2d& point)
{
    auto result = getPoint(point);

    if (!result)
    {
        const auto newVertex = makeVertex(point);
        result               = newVertex;
    }
    return Vertex(this, result->handle());
}

Point2d Dcel::point(Vertex vertex) const
{
    const auto v = getVertex(vertex);
    return v->location();
}

HalfEdge Dcel::edge(Vertex vertex) const
{
    const auto v = getVertex(vertex);

    if (v->edges().size())
        return HalfEdge(this, v->edges()[0]->handle());

    return HalfEdge();
}

HalfEdge Dcel::edge(Region region) const
{
    const auto r = getRegion(region);
    return HalfEdge(this, r->edge()->handle());
}

std::vector<Region> Dcel::regions() const
{
    std::vector<Region> result(m_regions.size());
    std::transform(m_regions.begin(), m_regions.end(), result.begin(),
                   [this](const std::pair<RegionHandle, RegionPtr>& region) { return Region(this, region.first); });
    return result;
}

Vertex Dcel::origin(HalfEdge edge) const
{
    const auto e = getEdge(edge);
    return Vertex(this, e->origin()->handle());
}

Region Dcel::region(HalfEdge edge) const
{
    const auto e = getEdge(edge);
    return Region(this, e->region()->handle());
}

HalfEdge Dcel::twin(HalfEdge edge) const
{
    const auto e = getEdge(edge);
    if (e->twin() == nullptr)
        return HalfEdge();

    return HalfEdge(this, e->twin()->handle());
}

HalfEdge Dcel::next(HalfEdge edge) const
{
    const auto e = getEdge(edge);
    return HalfEdge(this, e->next()->handle());
}

HalfEdge Dcel::previous(HalfEdge edge) const
{
    const auto e = getEdge(edge);
    return HalfEdge(this, e->prev()->handle());
}

HalfEdge Dcel::createRegion(Vertex v1, Vertex v2)
{
    const auto vertex1 = getVertex(v1);
    const auto vertex2 = getVertex(v2);

    const auto newRegion = makeRegion();
    const auto newEdge   = makeEdge(vertex1.get());

    newRegion->setEdge(newEdge.get());
    newEdge->setRegion(newRegion.get());

    splitEdge(newEdge.get(), vertex2.get());

    return HalfEdge(this, newEdge->handle());
}

HalfEdge Dcel::createRegion(HalfEdge e)
{
    const auto oldEdge = getEdge(e);

    // Can't create if there's already a region on the other side
    // of the edge
    if (oldEdge->twin())
        return HalfEdge();

    const auto newEdge = makeEdge(oldEdge->next()->origin());

    newEdge->setTwin(oldEdge.get());

    return HalfEdge(this, newEdge->handle());
}

HalfEdge Dcel::splitEdge(HalfEdge e, Vertex v)
{
    const auto vertex  = getVertex(v);
    const auto oldEdge = getEdge(e);

    return HalfEdge(this, splitEdge(oldEdge.get(), vertex.get())->handle());
}

Dcel::HalfEdgeData* Dcel::splitEdge(HalfEdgeData* oldEdge, VertexData* vertex)
{
    if (vertex == oldEdge->origin())
        return oldEdge;

    if (vertex == oldEdge->next()->origin())
        return oldEdge->next();

    const auto newEdge = makeEdge(vertex);

    newEdge->setNext(oldEdge->next());
    newEdge->setPrev(oldEdge);
    newEdge->setRegion(oldEdge->region());

    // If there's a twin, then it gets split also
    if (oldEdge->twin())
    {
        const auto oldTwin = oldEdge->twin();
        const auto newTwin = makeEdge(vertex);

        newTwin->setNext(oldTwin->next());
        newTwin->setPrev(oldTwin);
        newTwin->setRegion(oldTwin->region());
        newTwin->setTwin(oldEdge);

        oldTwin->setTwin(newEdge.get());
    }

    return newEdge.get();
}

bool Dcel::mergeRegions(HalfEdge r1e1, HalfEdge r2e1, HalfEdge r2e2, HalfEdge r1e2)
{
    auto r1edge1 = getEdge(r1e1).get();
    auto r1edge2 = getEdge(r1e2).get();

    auto r2edge1 = getEdge(r2e1).get();
    auto r2edge2 = getEdge(r2e2).get();

    auto r1 = r1edge1->region();
    auto r2 = r2edge1->region();

    if (r1edge1->region() != r1edge2->region())
        return false;

    if (r2edge1->region() != r2edge2->region())
        return false;

    if (r1edge1->region() == r2edge1->region())
        return false;

    // Step edges forward if they share points
    while (r2edge1->origin() == r1edge1->origin())
        r2edge1 = r2edge1->next();

    while (r1edge2->origin() == r2edge2->origin())
        r1edge2 = r1edge2->next();

    // Bounds for deleting edges on region 1; everything
    // between these will be removed. If they have twins, the
    // twins will remain
    auto r1UnusedBegin = r1edge1->next();
    auto r1UnusedEnd   = r1edge2;

    // Tracker for what r1 was pointing to previously,
    // in case we delete it
    auto r1Edge = r1->edge()->handle();

    while (r1UnusedBegin != r1UnusedEnd)
    {
        // Make sure to copy next before invalidating current
        auto next = r1UnusedBegin->next();

        m_edges.erase(r1UnusedBegin->handle());
        r1UnusedBegin = next;
    }

    auto r2UnusedBegin = r2edge2->next();
    auto r2UnusedEnd   = r2edge1;

    while (r2UnusedBegin != r2UnusedEnd)
    {
        // Make sure to copy next before invalidating current
        auto next = r2UnusedBegin->next();
        m_edges.erase(r2UnusedBegin->handle());
        r2UnusedBegin = next;
    }

    // Reassign existing edges to point to each other
    r1edge1->setNext(r2edge1);
    r2edge2->setNext(r1edge2);

    // Reassign all of the second region lines to the first
    // region
    while (r2edge1 != r1edge2)
    {
        r2edge1->setRegion(r1edge1->region());
        r2edge1 = r2edge1->next();
    }

    // Delete the second region
    m_regions.erase(r2->handle());

    // Check if we deleted the edge that region 1 pointed to
    if (m_edges.count(r1Edge) == 0)
        r1->setEdge(r1edge1);

    return true;
}

HalfEdge Dcel::splitRegion(HalfEdge e1, HalfEdge e2)
{
    auto edge1 = getEdge(e1).get();
    auto edge2 = getEdge(e2).get();

    // Check for obviously invalid cases
    if (edge1->region() != edge2->region() || edge1 == edge2 || edge1->next() == edge2)
        return HalfEdge();

    // Check if the region doesn't need to be split because there's
    // only one edge between the two given edges
    const bool edgeExists = edge1->next() == edge2->prev();

    // Can't create twin for an edge that already has a twin
    if ((edgeExists && edge1->next()->twin()))
        return HalfEdge();

    auto oldRegion = edge1->region();
    auto splitEdge = edgeExists ? edge1->next() : makeEdge(edge1->next()->origin()).get();
    auto twinEdge  = makeEdge(edge2->origin());
    auto newRegion = makeRegion();

    twinEdge->setRegion(newRegion.get());
    splitEdge->setTwin(twinEdge.get());
    newRegion->setEdge(twinEdge.get());

    if (!edgeExists)
    {
        //! \note Important that we assign the twin edge next/prev
        //! before reassigning edge1 and edge2's links

        twinEdge->setNext(edge1->next());
        twinEdge->setPrev(edge2->prev());

        splitEdge->setNext(edge2);
        splitEdge->setPrev(edge1);
        splitEdge->setRegion(edge1->region());

        // Assign the new region to all edges attached
        auto newEdgeIt = twinEdge->next();
        while (newEdgeIt != twinEdge.get())
        {
            // If the old region pointed to one of the edges
            // in the new region now, then reassign it
            if (newEdgeIt == oldRegion->edge())
                oldRegion->setEdge(edge1);

            newEdgeIt->setRegion(newRegion.get());
            newEdgeIt = newEdgeIt->next();
        }
    }
    // If we're not connecting the twin edge to something
    // specific before and after it, we need to at least
    // make sure it's got two endpoints to make a valid
    // segment
    else
    {
        auto afterTwin = makeEdge(splitEdge->origin()).get();

        twinEdge->setNext(afterTwin);
        twinEdge->setPrev(afterTwin);

        afterTwin->setRegion(newRegion.get());
    }

    return HalfEdge(this, splitEdge->handle());
}

std::pair<bool, std::string> Dcel::isValid() const
{
    std::stringstream errMsg;

    std::unordered_set<RegionData*> regionSet;
    std::unordered_set<HalfEdgeData*> edgeSet;
    std::unordered_set<VertexData*> vertexSet;

    for (const auto regionIt : m_regions)
    {
        const auto i      = regionIt.first;
        const auto region = regionIt.second;

        if (!region)
            return {false, (errMsg << "region " << i << " is null", errMsg.str())};

        if (!region->edge())
            return {false, (errMsg << "region " << i << " has null edge", errMsg.str())};

        regionSet.insert(region.get());
    }

    for (const auto edgeIt : m_edges)
    {
        const auto i    = edgeIt.first;
        const auto edge = edgeIt.second;

        if (!edge)
            return {false, (errMsg << "edge " << i << " is null", errMsg.str())};

        if (!edge->next() || !edge->prev())
            return {false, (errMsg << "edge " << i << " has null neighbor", errMsg.str())};

        if (!edge->origin())
            return {false, (errMsg << "edge " << i << " has null origin", errMsg.str())};

        if (!edge->region())
            return {false, (errMsg << "edge " << i << " has null region", errMsg.str())};

        edgeSet.insert(edge.get());
    }

    for (const auto vertexIt : m_vertices)
    {
        const auto i      = vertexIt.first;
        const auto vertex = vertexIt.second;

        if (!vertex)
            return {false, (errMsg << "vertex " << i << " is null", errMsg.str())};

        /*if (!vertex->edges().size())
            return {false, (errMsg << "vertex " << i << " has no edges", errMsg.str())};*/

        for (const auto edge : vertex->edges())
            if (edgeSet.count(edge) == 0)
                return {false, (errMsg << "vertex " << i << " points to unknown edge", errMsg.str())};

        vertexSet.insert(vertex.get());
    }

    for (const auto edgeIt : m_edges)
    {
        const auto i    = edgeIt.first;
        const auto edge = edgeIt.second;

        if (edgeSet.count(edge->next()) == 0 || edgeSet.count(edge->prev()) == 0)
            return {false, (errMsg << "edge " << i << " has unknown neighbor", errMsg.str())};

        if (edge->twin())
        {
            if (edgeSet.count(edge->twin()) == 0)
                return {false, (errMsg << "edge " << i << " has unknown twin", errMsg.str())};

            if (edge->twin()->twin() != edge.get())
                return {false, (errMsg << "edge " << i << " twin does not have edge " << i << " as twin", errMsg.str())};
        }

        if (vertexSet.count(edge->origin()) == 0)
            return {false, (errMsg << "edge " << i << " has unknown origin", errMsg.str())};

        if (regionSet.count(edge->region()) == 0)
            return {false, (errMsg << "edge " << i << " has unknown region", errMsg.str())};

        if (edge->region() != edge->next()->region() || edge->region() != edge->prev()->region())
            return {false, (errMsg << "edge " << i << " does not share region with neighbor", errMsg.str())};

        if (edge->next()->prev() != edge.get() || edge->prev()->next() != edge.get())
            return {false, (errMsg << "edge " << i << " is not properly linked to neighbors", errMsg.str())};
    }

    for (const auto regionIt : m_regions)
    {
        const auto i      = regionIt.first;
        const auto region = regionIt.second;

        if (edgeSet.count(region->edge()) == 0)
            return {false, (errMsg << "region " << i << " points to unknown edge", errMsg.str())};

        if (region->edge()->region() != region.get())
            return {false, (errMsg << "region " << i << " points to edge that does not point back", errMsg.str())};
    }

    return {true, ""};
}
}
}
