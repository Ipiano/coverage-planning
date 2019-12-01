TEMPLATE = app
unset(QT)
CONFIG += console c++11

include($$top_qmakedir/ccpp.pri)
include($$top_qmakedir/output_dirs.pri)
include($$top_qmakedir/gtest.pri)

PROJECT_ROOT = $$top_srcdir
TEST_ROOT=$$PROJECT_ROOT/gtests

# Not sure why, but I can't get regular regex.a to link
# so we're linking boost's version instead, which exposes
# the POSIX interface
LIBS += -lboost_regex

SOURCES += \
    $$TEST_ROOT/init-test.cpp \
    $$TEST_ROOT/uturn-test.cpp
