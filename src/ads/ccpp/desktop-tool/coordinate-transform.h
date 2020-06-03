#pragma once

#include "ads/ccpp/desktop-tool/typedefs.h"
#include "ads/ccpp/typedefs.h"

namespace ads
{
namespace ccpp
{
namespace desktop_tool
{
ccpp::geometry::Polygon2d project_polygon(const geometry::GeoPolygon2d<boost::geometry::degree>& p);
}
}
}
