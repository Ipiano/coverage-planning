#pragma once

#include <QString>
#include <QStringList>
#include <QFileInfo>

#include <boost/geometry/core/cs.hpp>

#include "ads/ccpp/desktop-tool/typedefs.h"

namespace ads {
namespace ccpp {
namespace desktop_tool {

class ImportShapeInterface
{
public:
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
    virtual std::pair<bool, geometry::GeoPolygon2d<boost::geometry::radian>> importShape(const QFileInfo& file) const = 0;
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
