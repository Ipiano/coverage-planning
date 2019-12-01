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

class GeojsonImporter : public QObject, public ImportShapeInterface
{
public:
    GeojsonImporter();

    virtual QStringList fileTypes() const override;
    virtual std::pair<bool, geometry::GeoPolygon2d<boost::geometry::radian>> importShape(const QFileInfo& fileInfo) const override;
};

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
