TEMPLATE = lib
unset(QT)
TARGET=ccpp
CONFIG += staticlib c++11

include($$top_qmakedir/boost.pri)
include($$top_qmakedir/output_dirs.pri)
include($$PWD/ccpp-lib-sources.pri)
