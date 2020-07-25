#ifndef EDGEDATA_H
#define EDGEDATA_H

#include "ads/algorithms/sweep-line/edge.h"
#include "ads/dcel/dcel.h"

namespace ads
{
namespace ccpp
{
namespace polygon_decomposer
{
namespace modified_trapezoidal
{
/*!
 * \brief Data structure that is attached to Edges in the sweep line algorithm
 *
 * The PolygonEdge type does not contain all of the information necessary for the
 * modified trapezoidal decomposition algorithm; but it does provide a mechanism to
 * store and retrieve an arbitrary data pointer in the edge. This is the type that
 * is created for each edge and stored during the algorithm.
 *
 * The two main purposes of this type are to provide logic to track which edges
 * have been processed, and track the DCEL handles that are being created and associated
 * with the edges as the algorithm progresses.
 */
class EdgeData
{
    algorithms::sweep_line::PolygonEdge* m_edge;

    dcel::HalfEdge m_halfEdge;
    dcel::Vertex m_firstVertex, m_secondVertex;

    bool m_processed;

  public:
    /*!
     * \brief Inits the edge data for a specific edge.
     *
     * During construction, the two points that define the edge
     * will be looked up in the dcel given; this will add them as
     * verticies in the dcel if they are not already present.
     *
     * The reference to the dcel is not held after construction.
     *
     * When EdgeData is constructed, the 'processed' state will be
     * set false, and will remain false until setProcessed() is invoked.
     *
     * When EdgeData is constructed, there will be no HalfEdge associated
     * with the data until one is assigned to the halfEdge() reference.
     *
     * \param edge Edge to construct data for
     * \param dcel Dcel being constructed in the decomposition algorithm
     */
    EdgeData(algorithms::sweep_line::PolygonEdge* edge, Dcel& dcel);

    /*!
     * \brief Gets the edge that was used to construct this data
     */
    algorithms::sweep_line::PolygonEdge* edge() const { return m_edge; }

    /*!
     * \brief Gets the dcel vertex associated with the first point of the edge
     */
    dcel::Vertex firstVertex() const { return m_firstVertex; }

    /*!
     * \brief Gets the dcel vertex associated with the second point of the edge
     */
    dcel::Vertex secondVertex() const { return m_secondVertex; }

    /*!
     * \brief Gets a reference to the dcel half edge currently associated with this data
     *
     * This is a non-const reference, so it can be assigned to to change the half edge
     * associated with this data.
     */
    dcel::HalfEdge& halfEdge() { return m_halfEdge; }

    /*!
     * \brief Gets the dcel half edge currently associated with this data
     */
    const dcel::HalfEdge& halfEdge() const { return m_halfEdge; }

    /*!
     * \brief Checks if the edge has been marked as processed
     */
    bool processed() const { return m_processed; }

    /*!
     * \brief Marks the edge as having been processed
     */
    void setProcessed() { m_processed = true; }

    /*!
     * \brief Gets the dcel vertex shared with the edge returned by leftEdge()
     */
    dcel::Vertex leftVertex() const;

    /*!
     * \brief Gets the dcel vertex shared with the edge returned by rightEdge()
     */
    dcel::Vertex rightVertex() const;

    /*!
     * \brief Gets the adjacent edge which is to the left of this edge
     *
     * This is similar to 'firstEdge()' with the notable
     * exception that it accounts for a vertical zig-zag situation and reverses
     * the result. Behavior is undefined if the edge is a leftmost edge of
     * its loop.
     *
     * \return The edge which is to the right of the current one
     */
    algorithms::sweep_line::PolygonEdge* leftEdge() const;

    /*!
     * \brief Gets the adjacent edge which is to the right of this edge
     *
     * This is similar to 'secondEdge()' with the notable
     * exception that it accounts for a vertical zig-zag situation and reverses
     * the result. Behavior is undefined if the edge is a rightmost edge of
     * its loop.
     *
     * \return The edge which is to the right of the current one
     */
    algorithms::sweep_line::PolygonEdge* rightEdge() const;

    /*!
     * \brief Checks if the edge is the first edge we will encounter on a loop
     *
     * When true, the edge is the 'bottom-left' edge of an inner or outer loop. This
     * means that no edge on the loop has an X coordinate < the first point of this
     * edge, and the adjacent edge sharing the first point is above this edge.
     */
    bool isLoopStart() const;

    /*!
     * \brief Checks if the edge is the last edge we have encountered on a loop
     *
     * This check is dependent on the processed() states of the adjacent edges.
     * The last edge on a loop is whichever edge is the last one processed -
     * that is, the two edges adjacent both have their processed() flag set,
     * but this one does not.
     *
     * This is guaranteed to be one of the right-most edges on the loop, but it
     * could be on either the top or bottom side of the shape. Due to the ordering
     * of edges, if the right-most edge on the shape is vertical, it is guaranteed
     * to be the end of the loop.
     */
    bool isLoopEnd() const;
};
}
}
}
}
#endif // EDGEDATA_H
