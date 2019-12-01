#include "mainwindow.h"
#include "ui_mainwindow.h"

#include "ads/ccpp/initial-cost/min-across-angles.hpp"
#include "ads/ccpp/turn-cost/u-shaped.h"

#include <QDebug>
#include <QFileDialog>
#include <QDir>
#include <QErrorMessage>
#include <QGraphicsPolygonItem>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

#include <boost/units/systems/si.hpp>
#include <boost/units/systems/angle/degrees.hpp>

namespace ads
{
namespace ccpp
{
namespace desktop_tool
{
namespace ui
{

namespace bg = boost::geometry;
namespace bu = boost::units;

typedef bu::quantity<bu::si::plane_angle> amnt_radians;
typedef bu::quantity<bu::degree::plane_angle> amnt_degrees;

typedef bg::model::d2::point_xy<double> CartesianPoint;
typedef bg::model::polygon<CartesianPoint> CartesianPoly;
typedef bg::ring_type<CartesianPoly>::type CartesianRing;

typedef ImportShapeInterface::GeoPoly GeoPoly;
typedef ImportShapeInterface::LonLatRad2d GeoPoint;
typedef bg::ring_type<GeoPoly>::type GeoRing;

CartesianPoint projectCartesian(GeoPoint point, const GeoPoint& reference);
CartesianRing projectCartesian(const GeoRing& ring, const GeoPoint& reference);
CartesianPoly projectCartesian(const GeoPoly& poly, const GeoPoint& reference);

QGraphicsItem* createItem(const CartesianPoly& poly);
QGraphicsItem* createItem(const CartesianRing& ring);
QGraphicsItem* createArrow(const CartesianPoint& origin, const double &length, const amnt_radians &angle);

class PointAverage
{
    GeoPoint m_average;
    uint32_t m_count;
public:
    PointAverage() : m_average(0, 0), m_count(0){}

    void operator()(const GeoPoint& pt)
    {
        bg::add_point(m_average, pt);
        m_count++;
    }

    GeoPoint average() const
    {
        auto result = m_average;
        bg::divide_value(result, m_count);
        return result;
    }
};


MainWindow::MainWindow(const QVector<std::shared_ptr<ImportShapeInterfaceFactory> > &shapeImporters, QWidget *parent) :
    QMainWindow(parent),
    m_ui(new Ui::MainWindow)
{
    m_ui->setupUi(this);

    m_scene = new QGraphicsScene(this);
    m_ui->graphicsView->setScene(m_scene);

    m_ui->graphicsView->setTransform(QTransform(1, 0, 0, 0, -1, 0, 0, 0, 1));

    connect(m_ui->action_loadShape, &QAction::triggered, this, &MainWindow::loadFile);

    connect(m_ui->checkBox_rawShape, &QCheckBox::clicked, this, &MainWindow::updateView);
    connect(m_ui->checkBox_initialDir, &QCheckBox::clicked, this, &MainWindow::updateView);

    for(const auto& importer : shapeImporters)
    {
        std::shared_ptr<ImportShapeInterface> shared(importer->create(), [importer](ImportShapeInterface* ptr){importer->destroy(ptr);});

        for(const QString& pattern : shared->fileTypes())
        {
            if(!m_importers.contains(pattern))
            {
                m_importers[pattern] = shared;
            }
            else
            {
                qWarning() << QString("Multiple plugins registered for file type '%1'; only one will be used").arg(pattern);
            }
        }
    }
}

MainWindow::~MainWindow()
{
}

void MainWindow::loadFile()
{
    if(m_importers.isEmpty())
    {
        QErrorMessage msg(this);
        msg.showMessage("No import plugins loaded");
        return;
    }

    QString filter;
    for(const QString& s : m_importers.keys())
        filter += s + ";;";
    filter = filter.left(filter.size()-2);

    QFileDialog dialog(this, "Load Shape", m_defaultFilePath, filter);
    dialog.exec();

    if(!dialog.selectedFiles().size())
        return;

    const auto chosen = QFileInfo(dialog.selectedFiles()[0]);
    if(!chosen.exists() || !chosen.isFile())
        return;

    const QString selectedFilter = dialog.selectedNameFilter();

    m_defaultFilePath = chosen.dir().path();

    const auto maybeShape = m_importers.value(selectedFilter)->importShape(chosen);
    if(!maybeShape.first)
    {
        QErrorMessage msg(this);
        msg.showMessage("Unable to load shape");
    }
    else
    {
        m_ui->label_fileName->setText("Loaded File: " + chosen.filePath());
        loadShape(maybeShape.second);
    }
}

void MainWindow::updateView()
{
    if(!m_rawShape)
        return;

    m_rawShape->setVisible(m_ui->checkBox_rawShape->checkState() == Qt::CheckState::Checked);
    m_initialDirArrow->setVisible(m_ui->checkBox_initialDir->checkState() == Qt::CheckState::Checked);
}

void MainWindow::loadShape(const GeoPoly& shape)
{
    m_scene->clear();

    const PointAverage centroidFinder = bg::for_each_point(shape, PointAverage());
    const GeoPoint centroid = centroidFinder.average();

    const auto shapeXY = projectCartesian(shape, centroid);

    ccpp::turn_cost::UShaped turnCost(1/0.5, 1/2., 1/2.);
    ccpp::initial_cost::MinAcrossAngles<ccpp::turn_cost::UShaped, CartesianPoly> initialCost(turnCost);
    const auto initialResult = initialCost(shapeXY);

    m_rawShape = createItem(shapeXY);
    const auto rect = m_rawShape->boundingRect();
    const auto diag = std::sqrt(rect.width()*rect.width() + rect.height()*rect.height());

    m_initialDirArrow = createArrow(bg::make_zero<CartesianPoint>(), 0.25*diag, initialResult.second);

    m_scene->addItem(m_rawShape);
    m_scene->addItem(m_initialDirArrow);

    m_ui->graphicsView->fitInView(m_scene->itemsBoundingRect(), Qt::AspectRatioMode::KeepAspectRatio);

    updateView();
}

CartesianPoint projectCartesian(GeoPoint point, const GeoPoint& reference)
{
    // TODO Maybe: Use better projection method
    bg::subtract_point(point, reference);

    auto result = bg::make<CartesianPoint>(static_cast<amnt_degrees>(bg::get<0>(point) * bu::si::radian).value(),
                                           static_cast<amnt_degrees>(bg::get<1>(point) * bu::si::radian).value());

    bg::multiply_value(result, 1e6);
    return result;
}

CartesianRing projectCartesian(const GeoRing& ring, const GeoPoint& reference)
{
    CartesianRing result;
    result.resize(ring.size());

    std::transform(ring.begin(), ring.end(), result.begin(), [reference](const GeoPoint& point)
                   {
                       return projectCartesian(point, reference);
                   });

    return result;
}

CartesianPoly projectCartesian(const GeoPoly& poly, const GeoPoint& reference)
{
    CartesianPoly result;
    result.inners().resize(poly.inners().size());

    result.outer() = projectCartesian(poly.outer(), reference);
    std::transform(poly.inners().begin(), poly.inners().end(), result.inners().begin(), [reference](const GeoRing& ring)
                   {
                       return projectCartesian(ring, reference);
                   });

    return result;
}

QGraphicsItem* createItem(const CartesianPoly& poly)
{
    auto shapeGroup = new QGraphicsItemGroup();
    shapeGroup->addToGroup(createItem(poly.outer()));
    for(const auto& ring : poly.inners())
    {
        shapeGroup->addToGroup(createItem(ring));
    }

    return shapeGroup;
}

QGraphicsItem* createItem(const CartesianRing& ring)
{
    QPolygonF qRing(int(ring.size()));

    std::transform(ring.begin(), ring.end(), qRing.begin(), [](const CartesianPoint& pt)
                   {
                       return QPointF(bg::get<0>(pt), bg::get<1>(pt));
                   });

    QGraphicsPolygonItem* ringGraphic = new QGraphicsPolygonItem(qRing);
    ringGraphic->setPen(QPen(QBrush(QColor()), 5));
    return ringGraphic;
}

QGraphicsItem* createArrow(const CartesianPoint& origin, const double& length, const amnt_radians& angle)
{
    QPainterPath path({bg::get<0>(origin), bg::get<1>(origin)});

    CartesianPoint tip = bg::make<CartesianPoint>(std::cos(angle.value()), std::sin(angle.value()));
    bg::multiply_value(tip, length);
    bg::add_point(tip, origin);

    const QPointF qTip(bg::get<0>(tip), bg::get<1>(tip));
    path.lineTo(qTip);

    const auto offset = static_cast<amnt_radians>(5*bu::degree::degree);

    CartesianPoint left = bg::make<CartesianPoint>(std::cos((angle+offset).value()), std::sin((angle+offset).value()));
    bg::multiply_value(left, length*0.8);
    bg::add_point(left, origin);

    path.lineTo({bg::get<0>(left), bg::get<1>(left)});
    path.lineTo(qTip);

    CartesianPoint right = bg::make<CartesianPoint>(std::cos((angle-offset).value()), std::sin((angle-offset).value()));
    bg::multiply_value(right, length*0.8);
    bg::add_point(right, origin);

    path.lineTo({bg::get<0>(right), bg::get<1>(right)});

    auto pathItem = new QGraphicsPathItem(path);
    pathItem->setPen(QPen(QBrush(QColor()), 5));

    return pathItem;
}

}
}
}
}
