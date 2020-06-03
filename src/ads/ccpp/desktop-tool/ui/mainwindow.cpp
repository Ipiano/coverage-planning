#include "mainwindow.h"
#include "ui_mainwindow.h"

#include "ads/ccpp/initial-cost/min-across-angles.hpp"
#include "ads/ccpp/turn-cost/u-shaped.h"
#include "ads/ccpp/polygon-decomposer/modified-trapezoidal.h"
#include "ads/ccpp/coordinate-transform.hpp"
#include "ads/ccpp/region-merger/region-merger.h"
#include "ads/ccpp/swath-and-region-producer/swath-and-region-producer.h"

#include "ads/ccpp/desktop-tool/coordinate-transform.h"
#include "ads/dcel/dcel.h"

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

// All coordinates are multiplied by this before drawing
// on the scene to get higher resolution/zooming capabilities
constexpr double SCENE_SCALING = 100;
template <class PointT> QPointF scenePoint(const PointT& p);
QGraphicsItem* drawItem(const ccpp::geometry::Polygon2d& poly, double borderWidth);
QGraphicsItem* drawItem(const ccpp::geometry::Ring2d& ring, double borderWidth);
QGraphicsItem* drawArrow(const ccpp::geometry::Point2d& origin, const double& length, const quantity::Radians& angle, const double width);
QGraphicsItem* drawRegions(const Dcel& dcel, double borderWidth);
std::array<QGraphicsItem*, 2>
drawMergedRegions(const std::vector<std::pair<ccpp::geometry::Ring2d, ccpp::geometry::MultiLine2d>>& regionGroups, double borderWidth);
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
    connect(m_ui->checkBox_unmergedSwaths, &QCheckBox::clicked, this, &MainWindow::updateView);
    connect(m_ui->checkBox_mergedRegions, &QCheckBox::clicked, this, &MainWindow::updateView);
    connect(m_ui->checkBox_mergedSwaths, &QCheckBox::clicked, this, &MainWindow::updateView);

    //connect(m_ui->spinBox_width, static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged), this, &MainWindow::recalculate);
    //connect(m_ui->spinBox_tolerance, static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged), this, &MainWindow::recalculate);
    connect(m_ui->button_recalculate, &QPushButton::clicked, this, &MainWindow::recalculate);

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
        m_currentShape = std::move(maybeShape.second);
        loadShape(m_currentShape);
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
    m_optimalSwathLines->setVisible(m_ui->checkBox_unmergedSwaths->checkState() == Qt::CheckState::Checked);
    m_mergedRegions->setVisible(m_ui->checkBox_mergedRegions->checkState() == Qt::CheckState::Checked);
    m_mergedSwathLines->setVisible(m_ui->checkBox_mergedSwaths->checkState() == Qt::CheckState::Checked);

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

void MainWindow::loadShape(const geometry::GeoPolygon2d<bg::degree>& shapeDegrees)
{
    m_scene->clear();

    auto shapeXY1 = project_polygon(shapeDegrees);
    ccpp::geometry::Polygon2d shapeXY2;

    ccpp::geometry::Point2d centroid;
    bg::centroid(shapeXY1, centroid);

    bg::strategy::transform::translate_transformer<double, 2, 2> translate(-bg::get<0>(centroid), -bg::get<1>(centroid));
    bg::transform(shapeXY1, shapeXY2, translate);

    bg::simplify(shapeXY2, shapeXY1, 0.1);

    const auto& shapeXY = shapeXY1;

    const auto shapeRect   = bg::return_envelope<ccpp::geometry::Box2d>(shapeXY);
    const auto shapeWidth  = std::abs(shapeRect.max_corner().x() - shapeRect.min_corner().x());
    const auto shapeHeight = std::abs(shapeRect.max_corner().y() - shapeRect.min_corner().y());
    const auto shapeDiag   = std::sqrt(shapeWidth + shapeWidth * shapeHeight + shapeHeight);

    // Works pretty well as an average width for viewing
    const auto borderWidth = std::max(1., 0.075 * shapeDiag);

    m_rawShape = drawItem(shapeXY, borderWidth);

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
        auto dcel = decomposer.decomposePolygon(adjustedShapeXY);

        ccpp::RegionMerger merger(dirCalculator);
        auto regionGroups = merger.mergeRegions(dcel);

        bool valid;
        std::string err;
        std::tie(valid, err) = dcel.isValid();
        if (!valid)
            throw std::runtime_error(err);

        ccpp::SwathAndRegionProducer swather(m_ui->spinBox_width->value());

        // Produce fake regions to get swaths the optimal directions
        std::vector<ccpp::interfaces::region_merger::MergeRegionGroup> singleRegionGroups;
        for (const auto regionHandle : dcel.regions())
        {
            ccpp::interfaces::region_merger::MergeRegion region;
            region.dcelRegion = regionHandle;

            const auto dirCost = dirCalculator.calculateOptimalDirectionAndCost(regionHandle);
            region.swathDir    = dirCost.first;

            singleRegionGroups.push_back({{region}});
        }

        auto unmergedSwathsAndRegions = swather.produceSwathsAndRegions(dcel, singleRegionGroups);
        auto mergedSwathsAndRegions   = swather.produceSwathsAndRegions(dcel, regionGroups);

        dcel.transform(invTransform);

        for (auto& swathsAndRegion : mergedSwathsAndRegions)
        {
            ccpp::geometry::Ring2d newRing;
            ccpp::geometry::MultiLine2d newLine;

            bg::transform(swathsAndRegion.first, newRing, invTransform);
            bg::transform(swathsAndRegion.second, newLine, invTransform);

            swathsAndRegion.first  = std::move(newRing);
            swathsAndRegion.second = std::move(newLine);
        }

        for (auto& swathsAndRegion : unmergedSwathsAndRegions)
        {
            ccpp::geometry::Ring2d newRing;
            ccpp::geometry::MultiLine2d newLine;

            bg::transform(swathsAndRegion.first, newRing, invTransform);
            bg::transform(swathsAndRegion.second, newLine, invTransform);

            swathsAndRegion.first  = std::move(newRing);
            swathsAndRegion.second = std::move(newLine);
        }

        // Draw the picture
        m_decomposition = drawRegions(dcel, borderWidth);
        m_scene->addItem(m_decomposition);

        auto unmergedRegionsAndLines = drawMergedRegions(unmergedSwathsAndRegions, borderWidth);
        m_optimalSwathLines          = unmergedRegionsAndLines[1];
        m_scene->addItem(m_optimalSwathLines);

        auto mergedRegionsAndLines = drawMergedRegions(mergedSwathsAndRegions, borderWidth);
        m_mergedRegions            = mergedRegionsAndLines[0];
        m_mergedSwathLines         = mergedRegionsAndLines[1];
        m_scene->addItem(m_mergedRegions);
        m_scene->addItem(m_mergedSwathLines);
    }
    catch (std::exception& ex)
    {
        qCritical() << "Exception:" << ex.what();
        m_decomposition     = new QGraphicsItemGroup();
        m_mergedRegions     = new QGraphicsItemGroup();
        m_mergedSwathLines  = new QGraphicsItemGroup();
        m_optimalSwathLines = new QGraphicsItemGroup();
        m_scene->addItem(m_decomposition);
        m_scene->addItem(m_optimalSwathLines);
        m_scene->addItem(m_mergedRegions);
        m_scene->addItem(m_mergedSwathLines);
    }

    m_initialDirArrow = drawArrow(bg::make_zero<ccpp::geometry::Point2d>(), 0.25 * shapeDiag, initialResult, borderWidth);

    m_scene->addItem(m_rawShape);
    m_scene->addItem(m_initialDirArrow);
    //m_scene->addItem(m_sweepPath);

    updateView();
}

template <> QPointF scenePoint<ccpp::geometry::Point2d>(const ccpp::geometry::Point2d& pt)
{
    return {bg::get<0>(pt) * SCENE_SCALING, bg::get<1>(pt) * SCENE_SCALING};
}

template <> QPointF scenePoint<dcel::Vertex>(const dcel::Vertex& pt)
{
    return scenePoint(pt.point());
}

QGraphicsItem* drawItem(const ccpp::geometry::Polygon2d& poly, double borderWidth)
{
    auto shapeGroup = new QGraphicsItemGroup();
    shapeGroup->addToGroup(drawItem(poly.outer(), borderWidth));
    for (const auto& ring : poly.inners())
    {
        shapeGroup->addToGroup(drawItem(ring, borderWidth));
    }

    return shapeGroup;
}

QGraphicsItem* drawItem(const ccpp::geometry::Ring2d& ring, double borderWidth)
{
    QPolygonF qRing(int(ring.size()));

    std::transform(ring.begin(), ring.end(), qRing.begin(), &scenePoint<ccpp::geometry::Point2d>);

    QGraphicsPolygonItem* ringGraphic = new QGraphicsPolygonItem(qRing);
    ringGraphic->setPen(QPen(QBrush(QColor(0, 0, 0, 90)), borderWidth));
    return ringGraphic;
}

QGraphicsItem* drawArrow(const ccpp::geometry::Point2d& origin, const double& length, const quantity::Radians& angle, const double width)
{
    QPainterPath path(scenePoint(origin));

    ccpp::geometry::Point2d tip = bg::make<ccpp::geometry::Point2d>(std::cos(angle.value()), std::sin(angle.value()));
    bg::multiply_value(tip, length);
    bg::add_point(tip, origin);

    const QPointF qTip(scenePoint(tip));
    path.lineTo(qTip);

    const auto offset = static_cast<quantity::Radians>(10 * bu::degree::degree);

    ccpp::geometry::Point2d left =
        bg::make<ccpp::geometry::Point2d>(std::cos((angle + offset).value()), std::sin((angle + offset).value()));
    bg::multiply_value(left, length * 0.8);
    bg::add_point(left, origin);

    path.lineTo(scenePoint(left));
    path.lineTo(qTip);

    ccpp::geometry::Point2d right =
        bg::make<ccpp::geometry::Point2d>(std::cos((angle - offset).value()), std::sin((angle - offset).value()));
    bg::multiply_value(right, length * 0.8);
    bg::add_point(right, origin);

    path.lineTo(scenePoint(right));

    auto pathItem = new QGraphicsPathItem(path);
    pathItem->setPen(QPen(QBrush(QColor("black")), width * 2));

    return pathItem;
}

QColor randomColor()
{
    std::uniform_int_distribution<uint8_t> rgbDist(125, 255);
    static std::mt19937_64 reng(std::chrono::system_clock::now().time_since_epoch().count());

    return QColor(rgbDist(reng), rgbDist(reng), rgbDist(reng));
}

QGraphicsItem* drawRegion(const dcel::Region& region, double borderWidth)
{
    QPolygonF poly;

    const auto firstEdge = region.edge();
    auto currEdge        = firstEdge;
    do
    {
        poly.push_back(scenePoint(currEdge.origin()));
    } while ((currEdge = currEdge.next()) != firstEdge);

    auto fill = randomColor();
    fill.setAlpha(125);

    QGraphicsPolygonItem* polyItem = new QGraphicsPolygonItem(poly);
    polyItem->setPen(QPen(QBrush(QColor("black")), borderWidth));
    polyItem->setBrush(QBrush(fill));
    polyItem->setVisible(true);

    return polyItem;
}

QGraphicsItem* drawRegions(const Dcel& dcel, double borderWidth)
{
    QGraphicsItemGroup* items = new QGraphicsItemGroup;

    for (const auto& region : dcel.regions())
    {
        items->addToGroup(drawRegion(region, borderWidth));
    }

    return items;
}

std::array<QGraphicsItem*, 2>
drawMergedRegions(const std::vector<std::pair<ccpp::geometry::Ring2d, ccpp::geometry::MultiLine2d>>& regionGroups, double borderWidth)
{
    QGraphicsItemGroup* regions = new QGraphicsItemGroup;
    QGraphicsItemGroup* lines   = new QGraphicsItemGroup;

    for (const auto& group : regionGroups)
    {
        QPolygonF qRing(int(group.first.size()));

        std::transform(group.first.begin(), group.first.end(), qRing.begin(), &scenePoint<ccpp::geometry::Point2d>);

        auto color = randomColor();
        color.setAlpha(125);

        QGraphicsPolygonItem* region = new QGraphicsPolygonItem(qRing);
        region->setPen(QPen(QBrush(QColor("black")), borderWidth));
        region->setBrush(QBrush(color));
        region->setVisible(true);
        regions->addToGroup(region);

        for (const auto& line : group.second)
        {
            QPainterPath path(scenePoint(line.front()));
            for (const auto& pt : line)
                path.lineTo(scenePoint(pt));

            QGraphicsPathItem* gPath = new QGraphicsPathItem(path);
            gPath->setPen(QPen(QBrush(QColor("black")), borderWidth));
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
