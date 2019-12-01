#pragma once

#include <QString>
#include <QStringList>
#include <QFileInfo>

#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/core/cs.hpp>

namespace ads {
namespace ccpp {
namespace desktop_tool {

class ImportShapeInterface
{
public:
    typedef boost::geometry::model::d2::point_xy<double, boost::geometry::cs::geographic<boost::geometry::radian>> LonLatRad2d;
    typedef boost::geometry::model::polygon<LonLatRad2d> GeoPoly;

    virtual ~ImportShapeInterface(){}

    // Return the list of file extentions that this
    // plugin can parse
    //
    // Each type should be of the form
    //  [name of type] ([ext], [ext]...])
    //
    //  e.g. Geojson (*.json, *.geojson)
    virtual QStringList fileTypes() const = 0;

    // Reads a file and converts it to a polygon
    // with 1 exterior shape and 0 or more interior ones
    //
    // Points should be in lon, lat order to keep with the x, y coord type
    //
    // Return [success, shape] where shape is empty if success is false
    virtual std::pair<bool, GeoPoly> importShape(const QFileInfo& file) const = 0;
};

class ImportShapeInterfaceFactory
{
public:
    virtual ~ImportShapeInterfaceFactory(){}

    virtual ImportShapeInterface* create() const = 0;
    virtual void destroy(ImportShapeInterface*) const = 0;
};

}
}
}

Q_DECLARE_INTERFACE(ads::ccpp::desktop_tool::ImportShapeInterfaceFactory, "ads.ccpp.desktop-tool.ImportShapeInterfaceFactory")
