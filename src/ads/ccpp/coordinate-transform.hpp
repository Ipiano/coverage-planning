#pragma once

#include <boost/qvm/mat.hpp>
#include <boost/geometry/strategies/transform/matrix_transformers.hpp>

#include <boost/qvm/mat.hpp>
#include <boost/qvm/vec.hpp>
#include <boost/qvm/map_vec_mat.hpp>
#include <boost/qvm/mat_operations.hpp>

#include "ads/ccpp/typedefs.h"

// GCC gives false positives on one of the qmv constructors below
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"

namespace ads
{
namespace ccpp
{
/*!
 * Creates a boost transform strategy which will move a shape to the origin and rotate it counterclockwise by an angle
 */
template <class T>
boost::geometry::strategy::transform::matrix_transformer<double, 2, 2> moveToOriginAndRotateCCWTransform(const T& shape,
                                                                                                         const quantity::Radians angle)
{
    typedef boost::qvm::mat<double, 3, 3> matrix;

    const auto centroid = boost::geometry::return_centroid<typename boost::geometry::point_type<T>::type>(shape);

    const boost::qvm::vec<double, 2> v1{-centroid.x(), -centroid.y()};
    const matrix tr = translation_mat(v1);

    constexpr boost::qvm::vec<double, 3> v2 = {0, 0, 1};
    const matrix rot                        = boost::qvm::rot_mat<3>(v2, angle.value());

    const auto op = rot * tr;
    return boost::geometry::strategy::transform::matrix_transformer<double, 2, 2>(op);
}
}
}

#pragma GCC diagnostic pop
