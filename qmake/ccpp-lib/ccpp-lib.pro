TEMPLATE = lib
unset(QT)

TARGET = ccpp
CONFIG += staticlib

include($$top_qmakedir/cpp-flags.pri)
include($$top_qmakedir/boost.pri)
include($$top_qmakedir/output-dirs.pri)
include($$PWD/ccpp-lib-sources.pri)
