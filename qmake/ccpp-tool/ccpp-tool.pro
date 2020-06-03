TEMPLATE = app
QT += gui widgets
CONFIG += console
TARGET = ccpp-tool

include($$top_qmakedir/cpp-flags.pri)
include($$top_qmakedir/ccpp.pri)
include($$top_qmakedir/geographiclib.pri)
include($$top_qmakedir/output-dirs.pri)

SOURCE_ROOT = $$top_srcdir/src
TOOL_ROOT = $$SOURCE_ROOT/ads/ccpp/desktop-tool

FORMS = $$TOOL_ROOT/ui/mainwindow.ui

SOURCES += \
    $$TOOL_ROOT/main.cpp \
    $$TOOL_ROOT/ui/mainwindow.cpp \
    $$TOOL_ROOT/coordinate-transform.cpp

HEADERS += \
    $$TOOL_ROOT/import-plugin.h \
    $$TOOL_ROOT/ui/mainwindow.h \
    $$TOOL_ROOT/coordinate-transform.h \
    $$TOOL_ROOT/typedefs.h
