#pragma once

#include "ads/ccpp/desktop-tool/import-plugin.h"

#include <QMainWindow>
#include <QMap>
#include <QVector>
#include <QGraphicsItem>
#include <QDir>

#include <memory>

namespace Ui {
class MainWindow;
}

namespace ads
{
namespace ccpp
{
namespace desktop_tool
{
namespace ui
{

class MainWindow : public QMainWindow
{
    Q_OBJECT

    QMap<QString, std::shared_ptr<ImportShapeInterface>> m_importers;

    QGraphicsScene* m_scene;

    QGraphicsItem* m_rawShape = nullptr;
    QGraphicsItem* m_initialDirArrow = nullptr;
    QGraphicsItem* m_sweepPath = nullptr;

    QString m_defaultFilePath = QDir::homePath();

    std::unique_ptr<Ui::MainWindow> m_ui;

    quantity::Radians m_sweepDir = units::Radian*0;

public:
    explicit MainWindow(const QVector<std::shared_ptr<ImportShapeInterfaceFactory> > &shapeImporters = {},
                        QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void loadFile();
    void loadShape(const geometry::GeoPolygon2d<boost::geometry::radian>& shape);
    void updateView();
};

}
}
}
}
