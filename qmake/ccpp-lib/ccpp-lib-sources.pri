SOURCE_ROOT = $$PWD/../../src
INCLUDEPATH += $$SOURCE_ROOT

CCPP_ROOT = $$SOURCE_ROOT/ads/ccpp

HEADERS += \
    $$CCPP_ROOT/ccpp.h \
    $$CCPP_ROOT/decomposition/polygon_decomposer_if.h \
    $$CCPP_ROOT/decomposition/boustrophedon_decomposer.h

SOURCES += \
