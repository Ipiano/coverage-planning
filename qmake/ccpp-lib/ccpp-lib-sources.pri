SOURCE_ROOT = $$top_srcdir/src
INCLUDEPATH += $$SOURCE_ROOT

CCPP_ROOT = $$SOURCE_ROOT/ads/ccpp

HEADERS += \
    $$CCPP_ROOT/ccpp.h \
    $$CCPP_ROOT/decomposition/polygon_decomposer_if.h \
    $$CCPP_ROOT/decomposition/boustrophedon_decomposer.h \
    $$CCPP_ROOT/dcel.hpp \
    $$CCPP_ROOT/initial-cost/initial-cost-concept.h \
    $$CCPP_ROOT/initial-cost/min-across-angles.hpp \
    $$CCPP_ROOT/turn-cost/turn-cost-concept.h \
    $$CCPP_ROOT/turn-cost/u-shaped.h \
    $$CCPP_ROOT/optimal-direction/optimal-direction-concept.h \
    $$CCPP_ROOT/optimal-direction/min-across-angles.hpp \
    $$CCPP_ROOT/typedefs.h


SOURCES += \
    $$CCPP_ROOT/turn-cost/u-shaped.cpp \
    $$CCPP_ROOT/dcel.cpp
