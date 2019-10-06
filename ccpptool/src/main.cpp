#include "ccpplib/ccpp.h"

#include <QApplication>

int main(int argc, char** argv)
{
    QApplication app(argc, argv);

    ccpp::CoveragePlanner planner;

    return app.exec();
}
