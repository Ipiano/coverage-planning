#pragma once

#include "ads/algorithms/sweep-line/geometry.h"

#include <memory>

namespace ads
{
namespace algorithms
{
namespace sweep_line
{
class PolygonSweepLineAlgorithm;

/*!
 * \brief Edge type for the sweep line algorithm
 *
 * Sorts the points contained according to the sweep line
 * sort requirement and allows storing and retrieving an
 * arbitary data pointer.
 */
class Edge
{
  public:
    /*!
     * \brief Contructs a sweep line edge with 2 points
     *
     * The points will be sorted, and can be retrieved in sorted
     * order with firstPoint() and secondPoint() respectively.
     *
     * \param p1
     * \param p2
     */
    Edge(const Point2d& p1, const Point2d& p2);

    /*!
     * \brief Sets the extra data associated with this Edge
     *
     * The data will be stored as a shared_ptr which is cast from
     * this one using static_pointer_cast to preserve the reference counter.
     *
     * \param data Pointer to data object
     */
    template <class DataT> void setData(std::shared_ptr<DataT> data) { m_data = std::static_pointer_cast<void>(data); }

    /*!
     * \brief Retrieves the extra data associated with this Edge, cast to the requested type
     *
     * The data is cast using static_pointer_cast.
     *
     * \return shared_ptr to the data stored
     */
    template <class DataT> std::shared_ptr<DataT> getData() { return std::static_pointer_cast<DataT>(m_data); }

    /*!
     * \brief Gets the point on the edge that is sorted first
     * \return
     */
    const Point2d& firstPoint() const;

    /*!
     * \brief Gets the point on the edge that is sorted second
     * \return
     */
    const Point2d& secondPoint() const;

    /*!
     * \brief Checks if a point is one of the two endpoints of this edge
     * \param point Point to check for in the edge
     * \return True if point is one of the endpoints of this edge
     */
    bool hasPoint(const Point2d& point) const;

    /*!
     * \brief Checks if the edge is vertical
     * \return True if the two points have the same X coordinate
     */
    bool isVertical() const;

    /*!
     * \brief Less Than operator used to sort edges for the sweep line algorithm
     *
     * An edge is considered less than another edge if
     * a. its left-most point is to the left of the other edge OR
     * b. its left-most point is at the same X coordinate as the left-most point of the other edge
     * but is below it OR
     * c. its left-most point is the same as the left-most point of the other edge and the angle
     * between the edge and a horizontal line is less (see angleLessThan())
     *
     * \param other Edge to compare to
     * \return True if this edge should be sorted before the other
     */
    bool operator<(const Edge& other) const;
    bool operator<(const Edge* other) const { return operator<(*other); }

    /*!
     * \brief Compares the angle of this edge to the angle of another edge
     *
     * The "angle" of an edge is the angle between it and a line on the X axis.
     * Angle values go from -PI (straight down) to +PI (straight up).
     *
     * \param other Edge to compare to
     * \return True if this edge has a lower angle than the other
     */
    bool angleLessThan(const Edge* other) const { return angleLessThan(*other); }
    bool angleLessThan(const Edge& other) const { return unitDotHorizontal() < other.unitDotHorizontal(); }

  private:
    std::shared_ptr<void> m_data;

    Point2d m_p1;
    Point2d m_p2;
    double m_angleSortValue;
    bool m_isVertical;

    double unitDotHorizontal() const;
};

/*!
 * \brief Edge type specifically for polygon sweep line algorithms
 *
 * This edge type can be linked with two edges after construction
 * in order to create a double-linked-list that tracks the order
 * of edges in the original polygon.
 *
 * This also allows the edge type to do some analysis of its surrounding
 * context to determine which edges are clockwise/counterclockwise, on the
 * left/right side, and whether the edge is the leftmost or rightmost edge
 * of a loop.
 */
class PolygonEdge : public Edge
{
  public:
    PolygonEdge(const Point2d& p1, const Point2d& p2);

    /*!
     * \brief Gets the direction of travel for this edge in the original shape
     * \return
     */
    bool isClockwise();

    //! Gets the previous edge in the loop, in whatever order it was originally defined
    PolygonEdge* previousEdge() const;

    //! Gets the next edge in the loop, in whatever order it was originally defined
    PolygonEdge* nextEdge() const;

    //! Gets the next edge traveling clockwise around the loop
    PolygonEdge* clockwiseEdge() const;

    //! Gets the next edge traveling counterclockwise around the loop
    PolygonEdge* counterclockwiseEdge() const;

    //! Gets the edge attached to this one at 'firstPoint()'
    PolygonEdge* firstEdge() const;

    //! Gets the edge attached to this one at 'secondPoint()'
    PolygonEdge* secondEdge() const;

    /*!
     * Checks if this edge is the vertical edge in between two edges
     * that go the 'same direction', meaning that the angle between those edges
     * is between 90 (a left turn) and 270 (a right turn).
     *
     * This image shows a couple examples of what it would look like
     * in extreme cases
     *
     * \verbatim
         *->-  ->-*
         |        |
         ^        ^
         |        |
      ->-*        *->-
      \endverbatim
     */
    bool isVerticalZigZag() const;

    /*!
     * \brief Less than operator to sort Polygon Edges slightly differently than regular Edges
     *
     * The PolygonEdge < operator is the same as the Edge < operator, except that it first
     * checks for a zig-zag. If the edge is vertical, and it's upper point is the second
     * point of the edge before and it's lower point is the first point of the next edge, like
     * so
     *
     * \verbatim
       ->-*
          |
          ^
          |
          *->-
      \endverbatim
     *
     * then the edge is a special case which is sorted before the edge after it. This guarantees
     * that when traversing the sorted edge list, the edges of the polygon as you travel around it
     * are in the order they are connected.
     *
     * \param other Edge to compare to
     * \return True if this edge is before the other
     */
    bool operator<(const PolygonEdge& other);
    bool operator<(const PolygonEdge* other) { return operator<(*other); }

  private:
    friend class PolygonSweepLineAlgorithm;

    PolygonEdge* m_nextEdge     = nullptr;
    PolygonEdge* m_previousEdge = nullptr;

    bool m_isClockwise;
    bool m_isVZigZag;

    // Rather than store a separate copy of each pointer for
    // each different interesting query on the edges, we just
    // track whether one of the possible outcomes maps to
    // nextEdge or not; that keeps storage down, but also keeps
    // lookups for various queries quick
    bool m_nextIsFirst;
    bool m_nextIsClockwise;

    /*!
     * \brief Sets the edges adjacent to build a linked list following the loop.
     *
     * Can only be set once. This has no effect on the adjacent edges. None of the
     * query methods on this edge can be called until this is invoked and all the results
     * are precalculated.
     *
     * \param previous The Edge before this one in the loop
     * \param clockwise The Edge after this one in the loop
     * \param isClockwise Indicator of if this edge is part of a clockwise loop or not
     */
    void setAdjacentEdges(PolygonEdge* previous, PolygonEdge* next, bool isClockwise);
};
}
}
}
