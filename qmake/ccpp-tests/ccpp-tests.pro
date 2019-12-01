TEMPLATE = app
unset(QT)
CONFIG += console c++11

include($$PWD/../ccpp.pri)
include($$PWD/../output_dirs.pri)
include($$PWD/../gtest.pri)

PROJECT_ROOT = $$PWD/../..
TEST_ROOT=$$PROJECT_ROOT/gtests

# Not sure why, but I can't get regular regex.a to link
# so we're linking boost's version instead, which exposes
# the POSIX interface
LIBS += -lboost_regex

SOURCES += \
    $$TEST_ROOT/init-test.cpp \
    $$TEST_ROOT/uturn-test.cpp
