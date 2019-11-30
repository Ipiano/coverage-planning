TEMPLATE = lib
unset(QT)
TARGET=ccpp
CONFIG += staticlib c++11

include($$PWD/../boost.pri)
include($$PWD/../output_dirs.pri)
include($$PWD/ccpp-lib-sources.pri)
