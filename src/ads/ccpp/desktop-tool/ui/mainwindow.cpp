#include "mainwindow.h"
#include "ui_mainwindow.h"

#include "ads/ccpp/initial-cost/min-across-angles.hpp"
#include "ads/ccpp/turn-cost/u-shaped.h"
#include "ads/ccpp/polygon-decomposer/modified-trapezoidal.h"
#include "ads/ccpp/coordinate-transform.hpp"
#include "ads/ccpp/region-merger/region-merger.h"
#include "ads/ccpp/swath-and-region-producer/swath-and-region-producer.h"

#include "ads/ccpp/desktop-tool/coordinate-transform.h"
#include "ads/ccpp/dcel.h"

#include <QDebug>
#include <QFileDialog>
#include <QDir>
#include <QErrorMessage>
#include <QGraphicsPolygonItem>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/strategies/cartesian/centroid_average.hpp>

#include <boost/units/systems/si.hpp>
#include <boost/units/systems/angle/degrees.hpp>

#include <random>
#include <unordered_set>
#include <chrono>

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

QGraphicsItem* drawItem(const ccpp::geometry::Polygon2d& poly);
QGraphicsItem* drawItem(const ccpp::geometry::Ring2d& ring);
QGraphicsItem* drawArrow(const ccpp::geometry::Point2d& origin, const double& length, const quantity::Radians& angle);
QGraphicsItem* drawRegions(const ccpp::DoublyConnectedEdgeList& dcel);
std::array<QGraphicsItem*, 2>
drawMergedRegions(const std::vector<std::pair<ccpp::geometry::Ring2d, ccpp::geometry::MultiLine2d>>& regionGroups);
//QGraphicsItem* createSweepPath(const std::vector<const dcel::const_half_edge_t*> edges);

QPointF makePoint(const ccpp::geometry::Point2d& pt)
{
    return {bg::get<0>(pt), bg::get<1>(pt)};
}

MainWindow::MainWindow(const QVector<std::shared_ptr<ImportShapeInterfaceFactory>>& shapeImporters, QWidget* parent)
    : QMainWindow(parent), m_ui(new Ui::MainWindow)
{
    m_ui->setupUi(this);

    m_scene = new QGraphicsScene(this);
    m_ui->graphicsView->setScene(m_scene);

    connect(m_ui->action_loadShape, &QAction::triggered, this, &MainWindow::loadFile);

    connect(m_ui->checkBox_rawShape, &QCheckBox::clicked, this, &MainWindow::updateView);
    connect(m_ui->checkBox_initialDir, &QCheckBox::clicked, this, &MainWindow::updateView);
    connect(m_ui->checkBox_sweepOrder, &QCheckBox::clicked, this, &MainWindow::updateView);
    connect(m_ui->checkBox_rotate, &QCheckBox::clicked, this, &MainWindow::updateView);
    connect(m_ui->checkBox_decomposition, &QCheckBox::clicked, this, &MainWindow::updateView);
    connect(m_ui->checkBox_merged, &QCheckBox::clicked, this, &MainWindow::updateView);
    connect(m_ui->checkBox_swaths, &QCheckBox::clicked, this, &MainWindow::updateView);

    connect(m_ui->spinBox_tolerance, static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged), this, &MainWindow::recalculate);

    m_defaultFilePath = "/home/ipiano/Documents/Code/MastersProject/test-files";

    for (const auto& importer : shapeImporters)
    {
        std::shared_ptr<ImportShapeInterface> shared(importer->create(), [importer](ImportShapeInterface* ptr) { importer->destroy(ptr); });

        for (const QString& pattern : shared->fileTypes())
        {
            if (!m_importers.contains(pattern))
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
    if (m_importers.isEmpty())
    {
        QErrorMessage msg(this);
        msg.showMessage("No import plugins loaded");
        return;
    }

    QString filter;
    for (const QString& s : m_importers.keys())
        filter += s + ";;";
    filter = filter.left(filter.size() - 2);

    QFileDialog dialog(this, "Load Shape", m_defaultFilePath, filter);
    dialog.exec();

    if (!dialog.selectedFiles().size())
        return;

    const auto chosen = QFileInfo(dialog.selectedFiles()[0]);
    if (!chosen.exists() || !chosen.isFile())
        return;

    const QString selectedFilter = dialog.selectedNameFilter();

    m_defaultFilePath = chosen.dir().path();
    m_currentShape.clear();

    const auto maybeShape = m_importers.value(selectedFilter)->importShape(chosen);
    if (!maybeShape.first)
    {
        QErrorMessage msg(this);
        msg.showMessage("Unable to load shape");
    }
    else
    {
        m_ui->label_fileName->setText("Loaded File: " + chosen.filePath());
        m_currentShape = maybeShape.second;
        loadShape(maybeShape.second);
    }
}

void MainWindow::recalculate()
{
    if (m_currentShape.outer().size() > 0)
    {
        loadShape(m_currentShape);
    }
}

void MainWindow::updateView()
{
    if (!m_rawShape)
        return;

    m_rawShape->setVisible(m_ui->checkBox_rawShape->checkState() == Qt::CheckState::Checked);
    m_initialDirArrow->setVisible(m_ui->checkBox_initialDir->checkState() == Qt::CheckState::Checked);
    //m_sweepPath->setVisible(m_ui->checkBox_sweepOrder->checkState() == Qt::CheckState::Checked);
    m_decomposition->setVisible(m_ui->checkBox_decomposition->checkState() == Qt::CheckState::Checked);
    m_mergedRegions->setVisible(m_ui->checkBox_merged->checkState() == Qt::CheckState::Checked);
    m_swathLines->setVisible(m_ui->checkBox_swaths->checkState() == Qt::CheckState::Checked);

    // Always flip scene upside down so +y is up
    QTransform transform(1, 0, 0, 0, -1, 0, 0, 0, 1);

    // Maybe rotate so that sweep direction is up
    if (m_ui->checkBox_rotate->checkState() == Qt::CheckState::Checked)
    {
        transform.rotate(static_cast<quantity::Degrees>(-m_sweepDir).value());
    }

    m_ui->graphicsView->setTransform(transform);
    m_ui->graphicsView->fitInView(m_scene->itemsBoundingRect(), Qt::AspectRatioMode::KeepAspectRatio);
}

void MainWindow::resizeEvent(QResizeEvent*)
{
    updateView();
}

void MainWindow::loadShape(const geometry::GeoPolygon2d<bg::radian>& shape)
{
    m_scene->clear();

    geometry::GeoPolygon2d<bg::degree> shapeDegrees;
    bg::transform(shape, shapeDegrees);

    auto shapeXY1 = cast_polygon<ccpp::geometry::Polygon2d>(shapeDegrees);
    ccpp::geometry::Polygon2d shapeXY2;

    ccpp::geometry::Point2d centroid;
    bg::centroid(shapeXY1, centroid);

    bg::strategy::transform::translate_transformer<double, 2, 2> translate(-bg::get<0>(centroid), -bg::get<1>(centroid));
    bg::transform(shapeXY1, shapeXY2, translate);

    bg::strategy::transform::scale_transformer<double, 2, 2> scale(1e6, 1e6);
    bg::transform(shapeXY2, shapeXY1, scale);

    const auto& scaledShapeXY = shapeXY1;
    const auto& shapeXY       = shapeXY1;
    m_rawShape                = drawItem(scaledShapeXY);

    ccpp::turn_cost::UShaped turnCost(1 / 0.5, 1 / 2., 1 / 2.);
    ccpp::optimal_direction::MinAcrossAngles dirCalculator(turnCost);

    ccpp::initial_cost::MinAcrossAngles initialCost(dirCalculator);
    const auto initialResult = initialCost.calculateInitialDirection(shapeXY);
    m_sweepDir               = initialResult;

    // Move shape to origin, and rotate so sweep dir is positive X direction
    const auto transform    = ccpp::moveToOriginAndRotateCCWTransform(shapeXY, -m_sweepDir);
    const auto invTransform = boost::geometry::strategy::transform::inverse_transformer<double, 2, 2>(transform);

    ccpp::geometry::Polygon2d adjustedShapeXY;
    boost::geometry::transform(shapeXY, adjustedShapeXY, transform);

    try
    {
        const auto tolerance = m_ui->spinBox_tolerance->value() * units::Degree;
        ccpp::polygon_decomposer::ModifiedTrapezoidal decomposer(tolerance);
        const auto dcel = decomposer.decomposePolygon(adjustedShapeXY);

        ccpp::RegionMerger merger(dirCalculator);
        auto regionGroups = merger.mergeRegions(dcel);

        bool valid;
        std::string err;
        std::tie(valid, err) = ccpp::dcel::is_valid(dcel);
        if (!valid)
            throw std::runtime_error(err);

        ccpp::SwathAndRegionProducer swather(50);
        auto swathsAndRegions = swather.produceSwathsAndRegions(dcel, regionGroups);

        //Rotate back to original orientation
        //and scale up for displaying
        for (const auto& point : dcel.vertices)
            bg::transform(point->location, point->location, invTransform);

        for (auto& swathsAndRegion : swathsAndRegions)
        {
            ccpp::geometry::Ring2d newRing;
            ccpp::geometry::MultiLine2d newLine;

            bg::transform(swathsAndRegion.first, newRing, invTransform);
            bg::transform(swathsAndRegion.second, newLine, invTransform);

            swathsAndRegion.first  = std::move(newRing);
            swathsAndRegion.second = std::move(newLine);
        }

        // Draw the picture
        m_decomposition = drawRegions(dcel);
        m_scene->addItem(m_decomposition);

        auto regionsAndLines = drawMergedRegions(swathsAndRegions);
        m_mergedRegions      = regionsAndLines[0];
        m_swathLines         = regionsAndLines[1];
        m_scene->addItem(m_mergedRegions);
        m_scene->addItem(m_swathLines);
    }
    catch (std::exception& ex)
    {
        qCritical() << "Exception:" << ex.what();
        m_decomposition = new QGraphicsItemGroup();
        m_mergedRegions = new QGraphicsItemGroup();
        m_swathLines    = new QGraphicsItemGroup();
        m_scene->addItem(m_decomposition);
        m_scene->addItem(m_mergedRegions);
        m_scene->addItem(m_swathLines);
    }

    const auto rect   = m_rawShape->boundingRect();
    const auto diag   = std::sqrt(rect.width() * rect.width() + rect.height() * rect.height());
    m_initialDirArrow = drawArrow(bg::make_zero<ccpp::geometry::Point2d>(), 0.25 * diag, initialResult);

    m_scene->addItem(m_rawShape);
    m_scene->addItem(m_initialDirArrow);
    //m_scene->addItem(m_sweepPath);

    updateView();
}

QGraphicsItem* drawItem(const ccpp::geometry::Polygon2d& poly)
{
    auto shapeGroup = new QGraphicsItemGroup();
    shapeGroup->addToGroup(drawItem(poly.outer()));
    for (const auto& ring : poly.inners())
    {
        shapeGroup->addToGroup(drawItem(ring));
    }

    return shapeGroup;
}

QGraphicsItem* drawItem(const ccpp::geometry::Ring2d& ring)
{
    QPolygonF qRing(int(ring.size()));

    std::transform(ring.begin(), ring.end(), qRing.begin(),
                   [](const ccpp::geometry::Point2d& pt) { return QPointF(bg::get<0>(pt), bg::get<1>(pt)); });

    QGraphicsPolygonItem* ringGraphic = new QGraphicsPolygonItem(qRing);
    ringGraphic->setPen(QPen(QBrush(QColor(0, 0, 0, 90)), 5));
    return ringGraphic;
}

QGraphicsItem* drawArrow(const ccpp::geometry::Point2d& origin, const double& length, const quantity::Radians& angle)
{
    QPainterPath path({bg::get<0>(origin), bg::get<1>(origin)});

    ccpp::geometry::Point2d tip = bg::make<ccpp::geometry::Point2d>(std::cos(angle.value()), std::sin(angle.value()));
    bg::multiply_value(tip, length);
    bg::add_point(tip, origin);

    const QPointF qTip(bg::get<0>(tip), bg::get<1>(tip));
    path.lineTo(qTip);

    const auto offset = static_cast<quantity::Radians>(10 * bu::degree::degree);

    ccpp::geometry::Point2d left =
        bg::make<ccpp::geometry::Point2d>(std::cos((angle + offset).value()), std::sin((angle + offset).value()));
    bg::multiply_value(left, length * 0.8);
    bg::add_point(left, origin);

    path.lineTo({bg::get<0>(left), bg::get<1>(left)});
    path.lineTo(qTip);

    ccpp::geometry::Point2d right =
        bg::make<ccpp::geometry::Point2d>(std::cos((angle - offset).value()), std::sin((angle - offset).value()));
    bg::multiply_value(right, length * 0.8);
    bg::add_point(right, origin);

    path.lineTo({bg::get<0>(right), bg::get<1>(right)});

    auto pathItem = new QGraphicsPathItem(path);
    pathItem->setPen(QPen(QBrush(QColor("black")), 3));

    return pathItem;
}

QColor randomColor()
{
    std::uniform_int_distribution<uint8_t> rgbDist(125, 255);
    static std::mt19937_64 reng(std::chrono::system_clock::now().time_since_epoch().count());

    return QColor(rgbDist(reng), rgbDist(reng), rgbDist(reng));
}

QPointF point(const dcel::vertex_t* vertex)
{
    return {vertex->location.x(), vertex->location.y()};
}

QGraphicsItem* drawRegion(const ccpp::dcel::region_t& region)
{
    QPolygonF poly;

    const auto firstEdge = region.edge;
    auto currEdge        = firstEdge;
    do
    {
        poly.push_back(point(currEdge->origin));
    } while ((currEdge = currEdge->next) != firstEdge);

    auto fill = randomColor();
    fill.setAlpha(125);

    QGraphicsPolygonItem* polyItem = new QGraphicsPolygonItem(poly);
    polyItem->setPen(QPen(QBrush(QColor("black")), 2));
    polyItem->setBrush(QBrush(fill));
    polyItem->setVisible(true);

    return polyItem;
}

QGraphicsItem* drawRegions(const ccpp::DoublyConnectedEdgeList& dcel)
{
    QGraphicsItemGroup* items = new QGraphicsItemGroup;

    for (const auto& region : dcel.regions)
    {
        items->addToGroup(drawRegion(*region));
    }

    return items;
}

std::array<QGraphicsItem*, 2>
drawMergedRegions(const std::vector<std::pair<ccpp::geometry::Ring2d, ccpp::geometry::MultiLine2d>>& regionGroups)
{
    QGraphicsItemGroup* regions = new QGraphicsItemGroup;
    QGraphicsItemGroup* lines   = new QGraphicsItemGroup;

    for (const auto& group : regionGroups)
    {
        QPolygonF qRing(int(group.first.size()));

        std::transform(group.first.begin(), group.first.end(), qRing.begin(),
                       [](const ccpp::geometry::Point2d& pt) { return QPointF(bg::get<0>(pt), bg::get<1>(pt)); });

        auto color = randomColor();
        color.setAlpha(125);

        QGraphicsPolygonItem* region = new QGraphicsPolygonItem(qRing);
        region->setPen(QPen(QBrush(QColor("black")), 2));
        region->setBrush(QBrush(color));
        region->setVisible(true);
        regions->addToGroup(region);

        for (const auto& line : group.second)
        {
            QPainterPath path({line.front().x(), line.front().y()});
            for (const auto& pt : line)
                path.lineTo(pt.x(), pt.y());

            QGraphicsPathItem* gPath = new QGraphicsPathItem(path);
            gPath->setPen(QPen(QBrush(QColor("black")), 2));
            gPath->setVisible(true);
            lines->addToGroup(gPath);
        }
    }

    return {regions, lines};
}

/*QGraphicsItem* createSweepPath(const std::vector<const dcel::const_half_edge_t*> edges)
{
    QPainterPath path(makePoint(edges[0]->origin->location));

    path.addEllipse(makePoint(edges[0]->origin->location), 10, 10);
    path.moveTo(makePoint(edges[0]->origin->location));

    for (const auto& edge : edges)
    {
        const auto& pt = edge->origin->location;
        path.lineTo(makePoint(pt));
    }

    auto pathItem = new QGraphicsPathItem(path);
    pathItem->setPen(QPen(QBrush(QColor(Qt::GlobalColor::red)), 2));

    return pathItem;
}*/
}
}
}
}
