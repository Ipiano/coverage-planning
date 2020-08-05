#include "ads/ccpp/quickopp.h"
#include "ads/ccpp/desktop-tool/ui/mainwindow.h"
#include "ads/ccpp/desktop-tool/import-plugin.h"

#include <boost/units/base_units/angle/radian.hpp>

#include <QApplication>
#include <QPluginLoader>
#include <QDirIterator>

using namespace ads::ccpp::desktop_tool;

/*!
 * \brief Loads all ImportShapeInterfaceFactory plugins found in a directory
 *
 * Tests every file in a directory to see if it can be loaded with a QPluginLoader
 * and cast to an ImportShapeInterfaceFactory.
 *
 * When a plugin loads successfully, it is added to the result as a shared pointer that
 * uses a custom deleter to hold a reference to the plugin loader. Once the interface
 * factory is deleted, the QPluginLoader used to load it will be unloaded.
 *
 * \param dir Directory to search for plugins
 * \return List of plugins found in the directory
 */
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

/*!
 * \brief Main application entrypoint
 *
 * The entrypoint to this tool does very little. It first attempts to load shape importer
 * plugins, and then creates and starts the main window, passing it the list of plugins found
 * so that it can use them to read data files.
 *
 * Plugins are searched for in two locations
 * * A 'shape-importers' directory in the same directory as the application binary
 * * A 'shape-importers' directory in the working directory
 * If both of these locations are the same, it is only searched once.
 *
 * \param argc
 * \param argv
 * \return 0 on successful execution and exit
 */
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
