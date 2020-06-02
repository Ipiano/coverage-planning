#include "geojson.h"

#include <boost/geometry.hpp>
#include <boost/units/systems/si.hpp>
#include <boost/units/systems/angle/degrees.hpp>

#include <QJsonDocument>
#include <QJsonValue>
#include <QJsonArray>
#include <QJsonObject>

#include <QDebug>

namespace bg = boost::geometry;

namespace ads
{
namespace ccpp
{
namespace desktop_tool
{
namespace import_plugins
{

//! Cleans a ring in an attempt to make bg::is_valid more likely to succeed
void clean(geometry::GeoRing2d<bg::degree>&);

//! Calculates unit vector in same direction as v
template <class Vector2T> Vector2T unit(Vector2T v);

GeojsonImporter::GeojsonImporter()
{
}

QStringList GeojsonImporter::fileTypes() const
{
    return {"GeoJson (*.geojson *.json)"};
}

bool isPolygon(const QJsonObject& object)
{
    return object["geometry"].toObject()["type"].toString() == "Polygon";
}

std::pair<bool, geometry::GeoRing2d<bg::degree>> convertPoly(const QJsonArray& coords)
{
    if (coords.size() < 3)
        return {false, {}};

    geometry::GeoRing2d<bg::degree> ring;
    ring.resize(uint32_t(coords.size()));

    std::transform(coords.begin(), coords.end(), ring.begin(), [](const QJsonValue& coordArr) {
        const QJsonArray coords = coordArr.toArray();
        return boost::geometry::make<geometry::GeoPoint2d<bg::degree>>(coords.at(0).toDouble(), coords.at(1).toDouble());
    });

    if (boost::geometry::distance(ring.front(), ring.back()) > 0.0001)
    {
        ring.push_back(ring.front());
    }

    if (ring.size() == 3)
        return {false, {}};

    return {true, std::move(ring)};
}

std::pair<bool, geometry::GeoPolygon2d<bg::degree>> GeojsonImporter::importShape(const QFileInfo& fileInfo) const
{
    QFile fileData(fileInfo.filePath());

    if (!fileData.open(QIODevice::ReadOnly))
    {
        qWarning() << "Failed to open file" << fileInfo.path();
        return {false, {}};
    }

    const auto byteData = fileData.readAll();
    fileData.close();

    QJsonParseError err;
    const auto jsonDoc = QJsonDocument::fromJson(byteData, &err);

    if (err.error != QJsonParseError::NoError)
    {
        qWarning() << QString("Error parsing %1: %2 (%3) @ %4")
                          .arg(fileData.fileName())
                          .arg(err.errorString())
                          .arg(int(err.error))
                          .arg(err.offset);
        return {false, {}};
    }

    if (!jsonDoc.isObject())
    {
        qWarning() << "File" << fileData.fileName() << "is not a json object file";
        return {false, {}};
    }

    const auto object = jsonDoc.object();
    QJsonObject featureObject;

    if (isPolygon(object))
    {
        featureObject = object;
    }
    else
    {
        QJsonArray featureList = object["features"].toArray();
        for (const QJsonValueRef feature : featureList)
        {
            if (isPolygon(feature.toObject()))
            {
                if (!featureObject.empty())
                {
                    qWarning() << "File" << fileData.fileName() << "contains multiple polygons";
                    return {false, {}};
                }
                featureObject = feature.toObject();
            }
        }
    }

    const QJsonObject geometry = featureObject["geometry"].toObject();
    const QJsonArray coords    = geometry["coordinates"].toArray();

    geometry::GeoPolygon2d<bg::degree> result;
    const auto maybeOuter = convertPoly(coords.at(0).toArray());
    if (!maybeOuter.first)
    {
        qWarning() << "File" << fileData.fileName() << "has invalid exterior polygon ring";
        return {false, {}};
    }

    result.outer() = std::move(maybeOuter.second);

    for (int i = 1; i < coords.size(); i++)
    {
        bool success;
        geometry::GeoRing2d<bg::degree> inner;

        std::tie(success, inner) = convertPoly(coords.at(i).toArray());
        if (!success)
        {
            qWarning() << "File" << fileData.fileName() << "has invalid inner polygon ring at index" << i;
            return {false, {}};
        }

        clean(inner);
        bg::correct(inner);

        // Ignore invalid holes inside the shape
        if (bg::is_valid(inner))
        {
            result.inners().push_back(std::move(inner));
        }
    }

    boost::geometry::correct(result);
    return {true, std::move(result)};
}

ImportShapeInterface* GeojsonImporterFactory::create() const
{
    return new GeojsonImporter();
}
void GeojsonImporterFactory::destroy(ImportShapeInterface* importer) const
{
    delete importer;
}

/*!
 * \brief Cleans a ring that was read so that it's more likely to pass a boost is_valid check
 *
 * This is done by removing any edges which are colinear or cause a spike. In other words
 * this removes points between edges with an angle of 0 or 180 degrees.
 *
 * \param[in, out] ring Ring to clean
 */
void clean(geometry::GeoRing2d<bg::degree>& ring)
{
    if (ring.size() < 3)
        return;

    size_t currInd = 0;

    while (currInd < ring.size() - 2)
    {
        const auto curr = ring.begin() + currInd;

        // Get next 3 consecutive points
        const auto& a = *curr;
        const auto& b = *(curr + 1);
        const auto& c = *(curr + 2);

        // Calculate the two edges
        auto ab = b;
        bg::subtract_point(ab, a);
        ab = unit(ab);

        auto bc = c;
        bg::subtract_point(bc, b);
        bc = unit(bc);

        // Find dot product
        const double dot = bg::dot_product(ab, bc);

        // If 1 or -1, then they are colinear or a spike, so we remove the
        // middle one
        if (std::abs(std::abs(dot) - 1) < 0.0000001)
        {
            ring.erase(curr + 1);

            // When we remove a point we need to step backwards if possible
            // because  there could be nested issues we need to solve.
            // (e.g. a spike off a spike, or a spike that breaks a colinear section)
            currInd--;
        }
        else
        {
            currInd++;
        }
    }
}

template <class Vector2T> Vector2T unit(Vector2T v)
{
    const auto a = bg::get<0>(v);
    const auto b = bg::get<1>(v);

    const auto mag = std::sqrt(a * a + b * b);
    bg::divide_value(v, mag);

    return v;
}
}
}
}
}
