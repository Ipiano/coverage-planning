TEMPLATE = app
QT += gui widgets
CONFIG += console c++11
TARGET = ccpp-tool

include($$top_qmakedir/ccpp.pri)
include($$top_qmakedir/output_dirs.pri)

SOURCE_ROOT = $$top_srcdir/src
TOOL_ROOT = $$SOURCE_ROOT/ads/ccpp/desktop-tool

SOURCES += \
    $$TOOL_ROOT/main.cpp
HEADERS += \
    $$TOOL_ROOT/import-plugin.h \
