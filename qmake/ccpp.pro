TEMPLATE = subdirs

SUBDIRS = \
    ccpp-lib \
    ccpp-lib-tests \
    ccpp-tool \
    geographiclib \
    geojson-plugin \

ccpp-lib.file = $$PWD/ccpp-lib/ccpp-lib.pro

ccpp-tool.file = $$PWD/ccpp-tool/ccpp-tool.pro
ccpp-tool.depends = ccpp-lib geographiclib

ccpp-lib-tests.file = $$PWD/ccpp-tests/ccpp-tests.pro
ccpp-lib-tests.depends = ccpp-lib

geographiclib.file = $$PWD/geographiclib/geographiclib.pro

geojson-plugin.file = $$PWD/ccpp-tool/plugins/geojson-plugin.pro

DISTFILES += $$PWD/../Doxyfile \
             $$PWD/../.gitignore
