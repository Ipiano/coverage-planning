TEMPLATE = lib
unset(QT)

TARGET = geographiclib
CONFIG += staticlib

include($$top_qmakedir/cpp-flags.pri)
include($$top_qmakedir/output-dirs.pri)

submodule_dir = $$top_srcdir/submodules/geographiclib

INCLUDEPATH += $$submodule_dir/include

HEADERS += $$files($$submodule_dir/include/GeographicLib/*.h) \
           $$files($$submodule_dir/include/GeographicLib/*.hpp)

SOURCES += $$files($$submodule_dir/src/*.cpp)
