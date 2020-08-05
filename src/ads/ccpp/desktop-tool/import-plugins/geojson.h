#pragma once

#include "ads/ccpp/desktop-tool/import-plugin.h"

namespace ads
{
namespace ccpp
{
namespace desktop_tool
{
namespace import_plugins
{

/*!
 * \brief Implementation of the ImportShapeInterface to import GeoJson files
 *
 * GeojsonImporter can only import data from geojson files containing exactly
 * one Polygon object which has exactly one outer ring. Attempting to import
 * any other geojson file will fail.
 */
class GeojsonImporter : public QObject, public ImportShapeInterface
{
  public:
    GeojsonImporter();

    virtual QStringList fileTypes() const override;
    virtual std::pair<bool, geometry::GeoPolygon2d<boost::geometry::degree>> importShape(const QFileInfo& fileInfo) const override;
};

/*!
 * \brief Qt Plugin implementation to provide the GeojsonImporter object
 */
class GeojsonImporterFactory : public QObject, public ImportShapeInterfaceFactory
{
    Q_OBJECT

    Q_PLUGIN_METADATA(IID "ads.ccpp.desktop-tool.ImportShapeInterfaceFactory")
    Q_INTERFACES(ads::ccpp::desktop_tool::ImportShapeInterfaceFactory)

  public:
    ImportShapeInterface* create() const override;
    void destroy(ImportShapeInterface* importer) const override;
};
}
}
}
}
