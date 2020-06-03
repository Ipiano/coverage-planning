win32: PRE_TARGETDEPS = $$OUT_PWD/../lib/geographiclib.lib
unix: PRE_TARGETDEPS = $$OUT_PWD/../lib/libgeographiclib.a

LIBS += -L$$OUT_PWD/../lib -lgeographiclib

submodule_dir = $$top_srcdir/submodules/geographiclib
INCLUDEPATH += $$submodule_dir/include
