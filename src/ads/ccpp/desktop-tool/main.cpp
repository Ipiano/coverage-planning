#include "ads/ccpp/ccpp.h"
#include "ads/ccpp/desktop-tool/ui/mainwindow.h"
#include "ads/ccpp/desktop-tool/import-plugin.h"

#include <boost/units/base_units/angle/radian.hpp>

#include <QApplication>
#include <QPluginLoader>
#include <QDirIterator>

using namespace ads::ccpp::desktop_tool;
QVector<std::shared_ptr<ImportShapeInterfaceFactory>> loadPlugins(QDir dir)
{
    QVector<std::shared_ptr<ImportShapeInterfaceFactory>> result;

    QDirIterator iter(dir.path(), QDir::Filter::Files);
    while (iter.hasNext())
    {
        QString path = iter.next();
        auto loader  = std::make_shared<QPluginLoader>(path);

        auto factory = qobject_cast<ImportShapeInterfaceFactory*>(loader->instance());
        if (factory)
        {
            result.push_back(std::shared_ptr<ImportShapeInterfaceFactory>(factory, [loader](ImportShapeInterfaceFactory* ptr) {
                delete ptr;
                loader->unload();
            }));
        }
    }
    return result;
}

int main(int argc, char** argv)
{
    QApplication app(argc, argv);

    auto plugins = loadPlugins(QApplication::applicationDirPath() + "/shape-importers");

    if (QDir(".").absolutePath() != QApplication::applicationDirPath())
    {
        plugins += loadPlugins(QDir("./shape-importers"));
    }

    ui::MainWindow window(plugins);
    window.show();

    return app.exec();
}
