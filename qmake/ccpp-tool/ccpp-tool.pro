TEMPLATE = app
QT += gui widgets
CONFIG += console c++11
TARGET = ccpp-tool

include($$PWD/../ccpp.pri)
include($$PWD/../output_dirs.pri)

SOURCE_ROOT = $$PWD/../../src
TOOL_ROOT = $$SOURCE_ROOT/ads/ccpp/desktop-tool

SOURCES += \
    $$TOOL_ROOT/main.cpp
