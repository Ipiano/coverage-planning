#include "mainwindow.h"
#include "ui_mainwindow.h"

#include "ads/ccpp/initial-cost/min-across-angles.hpp"
#include "ads/ccpp/turn-cost/u-shaped.h"

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

QGraphicsItem* createItem(const ccpp::geometry::Polygon2d& poly);
QGraphicsItem* createItem(const ccpp::geometry::Ring2d& ring);
QGraphicsItem* createArrow(const ccpp::geometry::Point2d& origin, const double& length, const quantity::Radians& angle);
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

    const auto maybeShape = m_importers.value(selectedFilter)->importShape(chosen);
    if (!maybeShape.first)
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
    if (!m_rawShape)
        return;

    m_rawShape->setVisible(m_ui->checkBox_rawShape->checkState() == Qt::CheckState::Checked);
    m_initialDirArrow->setVisible(m_ui->checkBox_initialDir->checkState() == Qt::CheckState::Checked);
    //m_sweepPath->setVisible(m_ui->checkBox_sweepOrder->checkState() == Qt::CheckState::Checked);

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

    const auto& shapeXY = shapeXY1;
    m_rawShape          = createItem(shapeXY);

    ccpp::turn_cost::UShaped turnCost(1 / 0.5, 1 / 2., 1 / 2.);
    ccpp::optimal_direction::MinAcrossAngles dirCalculator(turnCost);

    ccpp::initial_cost::MinAcrossAngles initialCost(dirCalculator);
    const auto initialResult = initialCost.calculateInitialDirection(shapeXY);
    m_sweepDir               = initialResult;

    const auto rect   = m_rawShape->boundingRect();
    const auto diag   = std::sqrt(rect.width() * rect.width() + rect.height() * rect.height());
    m_initialDirArrow = createArrow(bg::make_zero<ccpp::geometry::Point2d>(), 0.25 * diag, initialResult);

    /*const ccpp::DoublyConnectedEdgeList dcel(shapeXY);
    auto edges = dcel.edges(dcel.insideFace());
    ccpp::sortEdges(edges, initialResult);
    m_sweepPath = createSweepPath(edges);*/

    m_scene->addItem(m_rawShape);
    m_scene->addItem(m_initialDirArrow);
    //m_scene->addItem(m_sweepPath);

    updateView();
}

QGraphicsItem* createItem(const ccpp::geometry::Polygon2d& poly)
{
    auto shapeGroup = new QGraphicsItemGroup();
    shapeGroup->addToGroup(createItem(poly.outer()));
    for (const auto& ring : poly.inners())
    {
        shapeGroup->addToGroup(createItem(ring));
    }

    return shapeGroup;
}

QGraphicsItem* createItem(const ccpp::geometry::Ring2d& ring)
{
    QPolygonF qRing(int(ring.size()));

    std::transform(ring.begin(), ring.end(), qRing.begin(),
                   [](const ccpp::geometry::Point2d& pt) { return QPointF(bg::get<0>(pt), bg::get<1>(pt)); });

    QGraphicsPolygonItem* ringGraphic = new QGraphicsPolygonItem(qRing);
    ringGraphic->setPen(QPen(QBrush(QColor()), 5));
    return ringGraphic;
}

QGraphicsItem* createArrow(const ccpp::geometry::Point2d& origin, const double& length, const quantity::Radians& angle)
{
    QPainterPath path({bg::get<0>(origin), bg::get<1>(origin)});

    ccpp::geometry::Point2d tip = bg::make<ccpp::geometry::Point2d>(std::cos(angle.value()), std::sin(angle.value()));
    bg::multiply_value(tip, length);
    bg::add_point(tip, origin);

    const QPointF qTip(bg::get<0>(tip), bg::get<1>(tip));
    path.lineTo(qTip);

    const auto offset = static_cast<quantity::Radians>(5 * bu::degree::degree);

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
    pathItem->setPen(QPen(QBrush(QColor()), 5));

    return pathItem;
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
