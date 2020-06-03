#pragma once

#include "ads/dcel/geometry.h"
#include "ads/dcel/concepts.h"
#include "ads/dcel/handles.h"
#include "ads/dcel/objects.h"

#include <boost/geometry/index/rtree.hpp>

#include <memory>
#include <functional>
#include <unordered_map>

namespace ads
{
namespace dcel
{

//! Dcel class definition
class Dcel
{
    // Internal classes for tracking the structure
    class HalfEdgeData;

    //! Holds data about a specific vertex. This includes
    //! the location and which edges are using it. It is expected
    //! that vertices will not be used by a large number of edges.
    class VertexData
    {
        friend class HalfEdgeData;

        VertexHandle m_handle;

        std::vector<HalfEdgeData*> m_edges;
        Point2d m_location;

      public:
        VertexData(VertexHandle handle, const Point2d& location);

        //! \note Cannot be deleted while edges are assigned to it
        ~VertexData();

        VertexHandle handle() const;
        const Point2d& location() const;
        const std::vector<HalfEdgeData*>& edges() const;
    };

    //! Holds data about a region, and points to an edge on
    //! that region.
    class RegionData
    {
        friend class HalfEdgeData;

        RegionHandle m_handle;

        HalfEdgeData* m_edge = nullptr;

      public:
        RegionData(RegionHandle handle);

        RegionHandle handle() const;
        HalfEdgeData* edge() const;

        //! \brief Sets the edge associated with the region
        //! Does not modify the edge given
        //! \param edge not null
        void setEdge(HalfEdgeData* edge);
    };

    class HalfEdgeData
    {
        HalfEdgeHandle m_handle;

        HalfEdgeData* m_twin = nullptr;
        HalfEdgeData* m_next = nullptr;
        HalfEdgeData* m_prev = nullptr;
        VertexData* m_origin = nullptr;
        RegionData* m_region = nullptr;

      public:
        HalfEdgeData(HalfEdgeHandle handle, VertexData* origin);
        ~HalfEdgeData();

        HalfEdgeHandle handle() const;
        HalfEdgeData* next() const;
        HalfEdgeData* prev() const;
        HalfEdgeData* twin() const;
        VertexData* origin() const;
        RegionData* region() const;

        //! \brief Sets the next edge
        //! Sets the next edge and sets that edge's previous.
        //! Modifies the original next to point back to itself
        //! \param edge not null
        void setNext(HalfEdgeData* edge);

        //! \brief Sets the previous edge
        //! Sets the previous edge and sets that edge's next.
        //! Modifies the original previous to point forward to itself
        //! \param edge not null
        void setPrev(HalfEdgeData* edge);

        //! Sets the twin edge
        //! Sets this edge's twin, and the twin's twin.
        //! Sets the previous twin to have no twin
        //! \param twin may be null
        void setTwin(HalfEdgeData* edge);

        //! Sets the origin vertex
        //! Removes this edge from the previous origin,
        //! and adds it to the new one
        //! \param origin not null
        void setOrigin(VertexData* vertex);

        //! Sets the region
        //! Does not modify the region
        //! \param region not null
        void setRegion(RegionData* region);
    };

    typedef std::shared_ptr<HalfEdgeData> HalfEdgePtr;
    typedef std::shared_ptr<VertexData> VertexPtr;
    typedef std::shared_ptr<RegionData> RegionPtr;

    VertexHandle m_nextVertex;
    RegionHandle m_nextRegion;
    HalfEdgeHandle m_nextEdge;

    std::unordered_map<RegionHandle, RegionPtr, hash::Handle<RegionHandle>> m_regions;
    std::unordered_map<HalfEdgeHandle, HalfEdgePtr, hash::Handle<HalfEdgeHandle>> m_edges;
    std::unordered_map<VertexHandle, VertexPtr, hash::Handle<VertexHandle>> m_vertices;

    //! Indexer to use vertex type in r-tree
    struct VertexIndex
    {
        typedef Point2d result_type;
        const result_type& operator()(const VertexPtr& v) const { return v->location(); }
    };

    // Points are stored here so we can look up if there's already a vertex associated with
    // some point when it's added
    boost::geometry::index::rtree<VertexPtr, boost::geometry::index::quadratic<25>, VertexIndex> m_points;

    const double m_epsilon;

    // Offset for points when looking up if they already exist
    const Point2d m_epsilonVector;

    //! \throws InvalidHandleError if handle not valid
    RegionPtr getRegion(Region handle) const;

    //! \throws InvalidHandleError if handle not valid
    HalfEdgePtr getEdge(HalfEdge handle) const;

    //! \throws InvalidHandleError if handle not valid
    VertexPtr getVertex(Vertex v) const;

    //! Returns nullptr if not found
    VertexPtr getPoint(const Point2d& pt) const;

    RegionPtr makeRegion();
    HalfEdgePtr makeEdge(VertexData* origin);
    VertexPtr makeVertex(const Point2d& location);

    HalfEdgeData* splitEdge(HalfEdgeData* e, VertexData* v);

    template <class Functor> Functor forEachSegment(const RegionData& region, Functor f) const
    {
        auto it = region.edge();

        do
        {
            const boost::geometry::model::referring_segment<const Point2d> segment(it->origin()->location(),
                                                                                   it->next()->origin()->location());
            f(segment);
            it = it->next();
        } while (it != region.edge());

        return f;
    };

  public:
    /*!
     * \brief Initializes an empty DCEL and sets the epsilon value for point lookup
     * \param epsilon Side length of lookup square when adding points. Default 0.0001, which is < 1mm if the units are meters
     */
    Dcel(const double epsilon = 0.0001);
    ~Dcel();

    ///////////////////////////// Point Data Methods //////////////////////////////
    /*!
     * \brief Get the vertex associated with a point. If there is not one, it is added
     *
     * For any given location, there is only ever one vertex; so if the same point is
     * added multiple times, they will all map to the same vertex.
     *
     * \param point Location to get a vertex for
     * \return Handle for the vertex associated with the given point
     */
    Vertex vertex(const Point2d& point);

    /*!
     * \brief Get the vertex associated with a point.
     *
     * \param point Location to get a vertex for
     * \return Handle for the vertex associated with the given point
     *
     * \throws DoesNotExistError if the point has not been added
     */
    Vertex vertex(const Point2d& point) const;

    ///////////////////////////// Vertex Methods //////////////////////////////////
    /*!
     * \brief Get the location associated with a vertex
     *
     * \param vertex Handle for the vertex to query
     * \return Location associated with the given vertex
     *
     * \throws InvalidHandleError if given handle isn't valid
     */
    Point2d point(Vertex vertex) const;

    /*!
     * \brief Get the half-edge associated with a vertex
     *
     * \param vertex Handle for the vertex to query
     * \return Handle for the half-edge associated with the vertex
     *
     * \throws InvalidHandleError if given handle isn't valid
     */
    HalfEdge edge(Vertex vertex) const;

    /*!
     * \brief Check if a vertex exists in the dcel
     * \param vertex Vertex to look for
     * \return bool
     */
    bool hasVertex(Vertex vertex) const;

    ///////////////////////////// Region Methods //////////////////////////////////
    /*!
     * \brief Get the half-edge associated with a region
     *
     * \param region Handle for the region to query
     * \return Handle for the half-edge associated with the region
     *
     * \throws InvalidHandleError if given handle isn't valid
     */
    HalfEdge edge(Region region) const;

    /*!
     * \brief Gets a list of all the regions in the dcel
     * \return
     */
    std::vector<Region> regions() const;

    /*!
     * \brief Check if a region exists in the dcel
     * \param region Region to look for
     * \return bool
     */
    bool hasRegion(Region region) const;

    ///////////////////////////// Edge Methods ////////////////////////////////////
    /*!
     * \brief Get the origin vertex for a half-edge
     *
     * \param edge Handle for the edge to query
     * \return Handle for the vertex associated with the edge
     *
     * \throws InvalidHandleError if given handle isn't valid
     */
    Vertex origin(HalfEdge edge) const;

    /*!
     * \brief Get the region that a half-edge is part of
     *
     * \param edge Handle for the edge to query
     * \return Handle for the region associated with the edge
     *
     * \throws InvalidHandleError if given handle isn't valid
     */
    Region region(HalfEdge edge) const;

    /*!
     * \brief Get the twin of a half-edge
     *
     * \param edge Handle for the edge to query
     * \return Handle for the twin of the queried edge
     *
     * \throws InvalidHandleError if given handle isn't valid
     * \throws DoesNotExistError if the edge has no twin
     */
    HalfEdge twin(HalfEdge edge) const;

    /*!
     * \brief Get the edge that follows an edge
     *
     * \param edge Handle for the edge to query
     * \return Handle for the edge following the queried edge
     *
     * \throws InvalidHandleError if given handle isn't valid
     */
    HalfEdge next(HalfEdge edge) const;

    /*!
     * \brief Get the edge that precedes an edge
     *
     * \param edge Handle for the edge to query
     * \return Handle for the edge preceeding the queried edge
     *
     * \throws InvalidHandleError if given handle isn't valid
     */
    HalfEdge previous(HalfEdge edge) const;

    /*!
     * \brief Check if an edge exists in the dcel
     * \param edge Edge to look for
     * \return bool
     */
    bool hasEdge(HalfEdge edge) const;

    ///////////////////////////// Mutation Methods ////////////////////////////////
    /*!
     * \brief Creates a new region with two points defining an edge
     *
     * The half edges created will not have twins. Each will follow the
     * other.
     *
     * \param v1 First vertex in the region
     * \param v2 Second vertex in the region
     * \return The half edge originating at v1
     */
    HalfEdge createRegion(Vertex v1, Vertex v2);

    /*!
     * \brief Creates a new region which shares an edge with an existing region
     *
     * Creates a twin for the given edge and assigns it to a new region.
     *
     * If the given half edge already has a twin, no region will be created, and
     * the result will be an invalid half-edge.
     *
     * \param e Half edge to create a twin of
     * \return The newly-created twin edge, or an invalid edge if none was created.
     */
    HalfEdge createRegion(HalfEdge e);

    /*!
     * \brief Combine two regions by connecting edges on one to edges on the other
     *
     * The input is two pairs of edges; they represent the two merge points on the
     * regions. The first two edges will be connected such that r1e1 connects to
     * r2e1, and the second two will be connected such that r2e2 connects to r1e2.
     *
     * If r1e1 and r1e2 are not in the same region, or r2e1 and r2e2 are not in the same
     * region, or r1e1 and r2e1 are in the same region already, then the merge will fail.
     *
     * Any remaining edges from either region which are now no longer used will be
     * deleted. After the merge, the resulting region will have all of the edges
     * between r1e1 and r1e2, followed by all of the edges between r2e2 and r2e1
     *
     * Edges in region 2 will be reassigned to be in region 1.
     *
     * Runtime: Linear in the number of edges merged from region 2.
     *
     * \param r1e1 Edge on the first region to connect to the second region
     * \param r2e1 Edge on the second region to connect r1e1 to
     * \param r2e2 Edge on the second region to connect to the first region
     * \param r1e2 Edge on the first region to connect r2e2 to
     *
     * \return True if the merge was successful
     */
    bool mergeRegions(HalfEdge r1e1, HalfEdge r2e1, HalfEdge r2e2, HalfEdge r1e2);

    /*!
     * \brief Splits a region by connecting two edges on the region to each other with a new edge
     *
     * The created edge will originate at the end of the first edge, and end at the origin of the second
     * edge. This will result in a new region, which will be assigned to the twin of this new edge. Any other
     * edges connected to this twin will have their region re-assigned to the new one.
     *
     * If there is already exactly one edge between e1 and e2, then this behaves as createRegion when
     * passed that edge - a twin will be created for that existing edge. Unlike createRegion, the existing
     * edge will be returned in this case.
     *
     * The following cases are invalid and will return an invalid edge
     * * e1 is in a different region than e2
     * * e1 and e2 are the same
     * * e2 directly follows e1
     * * There is exactly one edge between e1 and e2, and it already has a twin
     *
     * \param e1 Edge to start split at
     * \param e2 Edge to end split at
     * \return The edge between e1 and e2, its twin is on the new region
     */
    HalfEdge splitRegion(HalfEdge e1, HalfEdge e2);

    /*!
     * \brief Splits a half edge by inserting a point in the middle of it.
     *
     * Returns the new edge created, which will follow the given edge and be part of
     * the same region.
     *
     * If the given edge has a twin, then that twin will also be split at the
     * same point
     *
     * If this would create a 0-length edge, then no new edge will be inserted.
     * Instead, the existing edge (either the given edge e, or the edge following it)
     * which has its origin at the given vertex will be returned.
     *
     * \param e Edge to insert after
     * \param v Vertex to originate new edge at
     * \return The half edge with v as its origin
     */
    HalfEdge splitEdge(HalfEdge e, Vertex v);

    ///////////////////////////// Utility Methods /////////////////////////////////
    /*! \brief Executes a functor for each segment in a specific region and returns the functor
     *
     * Behaves similarly to boost::for_each_segment
     *
     * \tparam Functor A functor object satisfying ForEachFunctorConcept.
     *
     * \throws InvalidHandleError if handle isn't valid
     *
     * \returns A copy of the functor object which has been applied to all segments
     */
    template <class Functor>
    BOOST_CONCEPT_REQUIRES(((ForEachFunctorConcept<Functor>)), (Functor))
    forEachSegment(Region region, Functor f) const
    {
        const auto regionPtr = getRegion(region);

        return forEachSegment<Functor>(*regionPtr, std::move(f));
    }

    /*! \brief Executes a functor for each segment in a specific region and returns the functor
     *
     * Behaves similarly to boost::for_each_segment
     *
     * \tparam Functor A functor object satisfying ForEachFunctorConcept
     *
     * \returns A copy of the functor object which has been applied to all segments
     */
    template <class Functor> BOOST_CONCEPT_REQUIRES(((ForEachFunctorConcept<Functor>)), (Functor)) forEachSegment(Functor f) const
    {
        for (const auto& regionIt : m_regions)
            f = std::move(forEachSegment<Functor>(*regionIt.second, std::move(f)));

        return f;
    }

    /*!
     * \brief Transforms the dcel by moving every point according to some strategy
     *
     * \tparam Strategy Strategy type defining how to move the dcel. Must satisfy TransformStrategyConcept.
     * The strategy should behave similarly to boost::geometry::transform strategies, where invoking 'apply(p1, p2)'
     * will modify p2 by transforming p1.
     *
     * \param s Strategy to apply
     */
    template <class Strategy> BOOST_CONCEPT_REQUIRES(((TransformStrategyConcept<Strategy>)), (void)) transform(Strategy s)
    {
        // Since all the points are changing, we need to re-insert
        // them in to the rtree
        m_points.clear();

        for (auto vertex : m_vertices)
        {
            // Calculate where the point moves to
            Point2d newPoint;
            s.apply(vertex.second->location(), newPoint);

            // Reassign it
            *vertex.second = VertexData(vertex.second->handle(), newPoint);

            // Insert back into rtree
            m_points.insert(vertex.second);
        }
    }

    std::pair<bool, std::string> isValid() const;
};

template <class Functor> Functor Region::forEachSegment(Functor f) const
{
    return dcel() ? dcel()->forEachSegment(*this, f) : f;
}
}

typedef dcel::Dcel Dcel;
}
