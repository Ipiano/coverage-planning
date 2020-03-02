TEMPLATE = lib
unset(QT)
TARGET=ccpp
CONFIG += staticlib c++11

include($$top_qmakedir/boost.pri)
include($$top_qmakedir/output-dirs.pri)
include($$PWD/ccpp-lib-sources.pri)

QMAKE_CXXFLAGS += -Wno-deprecated-copy
