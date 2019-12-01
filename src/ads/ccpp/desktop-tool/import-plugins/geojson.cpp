#include "geojson.h"

#include <boost/geometry.hpp>

#include <QJsonDocument>
#include <QJsonValue>
#include <QJsonArray>
#include <QJsonObject>

#include <QDebug>

namespace ads
{
namespace ccpp
{
namespace desktop_tool
{
namespace import_plugins
{

GeojsonImporter::GeojsonImporter()
{

}

QStringList GeojsonImporter::fileTypes()
{
    return {"Geojson (*.geojson, *.json)"};
}

bool isPolygon(const QJsonObject& object)
{
    return object["geometry"].toObject()["type"].toString() == "Polygon";
}

std::pair<bool, boost::geometry::ring_type<GeojsonImporter::GeoPoly>::type> convertPoly(const QJsonArray& coords)
{
    if(coords.size() < 3)
        return {false, {}};

    boost::geometry::ring_type<GeojsonImporter::GeoPoly>::type ring;
    ring.resize(uint32_t(coords.size()));

    std::transform(coords.begin(), coords.end(), ring.begin(), [](const QJsonValue& coordArr)
                   {
                       const QJsonArray coords = coordArr.toArray();
                       return boost::geometry::make<GeojsonImporter::LonLatRad2d>
                            (coords.at(0).toDouble(), coords.at(1).toDouble());
                   });

    if(boost::geometry::distance(ring.front(), ring.back()) > 0.0001)
    {
        ring.push_back(ring.front());
    }

    if(ring.size() == 3)
        return {false, {}};

    return {true, std::move(ring)};
}

std::pair<bool, GeojsonImporter::GeoPoly> GeojsonImporter::importShape(const QFileInfo& fileInfo)
{
    QFile fileData(fileInfo.path());

    if(!fileData.open(QIODevice::ReadOnly))
    {
        qWarning() << "Failed to open file" << fileInfo.path();
        return {false, {}};
    }

    const auto byteData = fileData.readAll();
    fileData.close();

    QJsonParseError err;
    const auto jsonDoc = QJsonDocument::fromJson(byteData, &err);

    if(err.error != QJsonParseError::NoError)
    {
        qWarning() << QString("Error parsing %1: %2 (%3) @ %4")
                          .arg(fileData.fileName())
                          .arg(err.errorString())
                          .arg(int(err.error))
                          .arg(err.offset);
        return {false, {}};
    }

    if(!jsonDoc.isObject())
    {
        qWarning() << "File" << fileData.fileName() << "is not a json object file";
        return {false, {}};
    }

    const auto object = jsonDoc.object();
    QJsonObject featureObject;

    if(isPolygon(object))
    {
        featureObject = object;
    }
    else
    {
        QJsonArray featureList = object["features"].toArray();
        for(const QJsonValueRef feature : featureList)
        {
            if(isPolygon(feature.toObject()))
            {
                if(!featureObject.empty())
                {
                    qWarning() << "File" << fileData.fileName() << "contains multiple polygons";
                    return {false, {}};
                }
                featureObject = feature.toObject();
            }
        }
    }

    const QJsonObject geometry = featureObject["geometry"].toObject();
    const QJsonArray coords = geometry["coordinates"].toArray();

    GeoPoly result;
    const auto maybeOuter = convertPoly(coords.at(0).toArray());
    if(!maybeOuter.first)
    {
        qWarning() << "File" << fileData.fileName() << "has invalid exterior polygon ring";
        return {false, {}};
    }

    result.outer() = std::move(maybeOuter.second);

    for(int i=1; i < coords.size(); i++)
    {
        const auto maybeInner = convertPoly(coords.at(i).toArray());
        if(!maybeInner.first)
        {
            qWarning() << "File" << fileData.fileName() << "has invalid inner polygon ring at index" << i;
            return {false, {}};
        }

        result.inners().push_back(std::move(maybeInner.second));
    }

    boost::geometry::correct(result);
    return {true, std::move(result)};
}

ImportShapeInterface* GeojsonImporterFactory::create()
{
    return new GeojsonImporter();
}
void GeojsonImporterFactory::destroy(ImportShapeInterface* importer)
{
    delete importer;
}

}
}
}
}
