TEMPLATE = lib
QT -= gui widgets
CONFIG += plugin
TARGET = geojson

include($$top_qmakedir/cpp-flags.pri)
include($$top_qmakedir/output-dirs.pri)
include($$top_qmakedir/boost.pri)

DESTDIR = $$top_builddir/bin/shape-importers

SOURCE_ROOT = $$top_srcdir/src
PLUGINS_ROOT = $$SOURCE_ROOT/ads/ccpp/desktop-tool/import-plugins

INCLUDEPATH += $$SOURCE_ROOT

SOURCES += \
    $$PLUGINS_ROOT/geojson.cpp

HEADERS += \
    $$PLUGINS_ROOT/geojson.h
