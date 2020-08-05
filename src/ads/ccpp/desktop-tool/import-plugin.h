#pragma once

#include <QString>
#include <QStringList>
#include <QFileInfo>

#include <boost/geometry/core/cs.hpp>

#include "ads/ccpp/desktop-tool/typedefs.h"

namespace ads
{
namespace ccpp
{
namespace desktop_tool
{

/*!
 * \brief Import interface that can read some file type and return geographic polygon data contained
 *
 * Each importer supports some set of file types, denoted by the extensions. Any file with one of
 * these extensions is understood by the importer and may be passed into the importShape() method
 * to attempt to get geographic polygon data out of it.
 */
class ImportShapeInterface
{
  public:
    virtual ~ImportShapeInterface() {}

    /*!
     * \brief Return the list of file extentions that this
     * importer can understand
     *
     * Each type should be of the form
     *  [name of type] ([ext], [ext]...])
     *
     *  e.g. Geojson (*.json, *.geojson)
     *
     * This makes the values appropriate for user-display and
     * use in a QFileDialog.
     */
    virtual QStringList fileTypes() const = 0;

    /*!
     * \brief Reads a file and attempts to convert it to a geographic polygon
     * with 1 exterior shape and 0 or more interior ones
     *
     * Points are returned in lon, lat order to keep with the x, y coord type
     *
     * \param file File info about the file to read
     * \return Pair [success, shape] where shape is empty if success is false
     */
    virtual std::pair<bool, geometry::GeoPolygon2d<boost::geometry::degree>> importShape(const QFileInfo& file) const = 0;
};

/*!
 * \brief Qt Plugin interface for shape importers
 *
 * This is registered using Q_DECLARE_INTERFACE and the interface designator
 * "ads.ccpp.desktop-tool.ImportShapeInterfaceFactory". This allows it to
 * be inherited for the creation of plugins that provide import functionalities
 * for different file types.
 */
class ImportShapeInterfaceFactory
{
  public:
    virtual ~ImportShapeInterfaceFactory() {}

    //! Creates a new importer. The importer should be deleted only using destroy()
    virtual ImportShapeInterface* create() const = 0;

    //! Deletes an existing importer created via create()
    virtual void destroy(ImportShapeInterface*) const = 0;
};
}
}
}

Q_DECLARE_INTERFACE(ads::ccpp::desktop_tool::ImportShapeInterfaceFactory, "ads.ccpp.desktop-tool.ImportShapeInterfaceFactory")
