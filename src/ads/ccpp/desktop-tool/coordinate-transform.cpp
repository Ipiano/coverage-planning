#include "coordinate-transform.h"

#include <GeographicLib/LocalCartesian.hpp>

namespace bg = boost::geometry;

namespace ads
{
namespace ccpp
{
namespace desktop_tool
{

struct point_sum
{
    double sum_x = 0, sum_y = 0;
    double count = 0;

    template <class PointT> void operator()(const PointT& p)
    {
        sum_x += bg::get<0>(p);
        sum_y += bg::get<1>(p);
        count++;
    };
};

struct local_cartesian_strategy
{
    GeographicLib::LocalCartesian proj;
    local_cartesian_strategy(const ccpp::geometry::Point2d& ref) { proj = GeographicLib::LocalCartesian(bg::get<1>(ref), bg::get<0>(ref)); }
    bool apply(const geometry::GeoPoint2d<bg::degree>& in, ccpp::geometry::Point2d& out) const
    {
        double x, y, _;
        proj.Forward(bg::get<1>(in), bg::get<0>(in), 0, x, y, _);
        out = bg::make<ccpp::geometry::Point2d>(x, y);

        return true;
    }
};

ccpp::geometry::Polygon2d project_polygon(const geometry::GeoPolygon2d<bg::degree>& p)
{
    const auto sum_pt     = bg::for_each_point(p, point_sum());
    const auto average_pt = bg::make<ccpp::geometry::Point2d>(sum_pt.sum_x / sum_pt.count, sum_pt.sum_y / sum_pt.count);

    ccpp::geometry::Polygon2d result;
    bg::transform(p, result, local_cartesian_strategy(average_pt));

    return result;
}
}
}
}
