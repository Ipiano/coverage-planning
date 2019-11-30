TEMPLATE = app
unset(QT)
CONFIG += console c++11

include($$PWD/../ccpp.pri)
include($$PWD/../output_dirs.pri)
include($$PWD/../gtest.pri)

PROJECT_ROOT = $$PWD/../..
TEST_ROOT=$$PROJECT_ROOT/gtests

SOURCES += \
    $$TEST_ROOT/init_test.cpp
