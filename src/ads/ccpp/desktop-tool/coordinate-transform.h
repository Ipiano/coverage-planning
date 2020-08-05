#pragma once

#include "ads/ccpp/desktop-tool/typedefs.h"
#include "ads/ccpp/typedefs.h"

namespace ads
{
namespace ccpp
{
namespace desktop_tool
{
/*!
 * \brief Projects a geographic polygon onto a local cartesian coordinate system
 *
 * The reference for the coordinate system is chosen as the centroid of the
 * polygon given.
 *
 * Point values are expected to be lon, lat order to keep with x, y convention.
 *
 * \param p Polygon to project
 * \return A polygon projected onto a cartesian coordinate plane
 */
ccpp::geometry::Polygon2d project_polygon(const geometry::GeoPolygon2d<boost::geometry::degree>& p);
}
}
}
