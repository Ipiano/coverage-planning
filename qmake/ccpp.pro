TEMPLATE = subdirs

SUBDIRS = \
    ccpp-lib \
    ccpp-lib-tests \
    ccpp-tool

ccpp-lib.file = $$PWD/ccpp-lib/ccpp-lib.pro

ccpp-tool.file = $$PWD/ccpp-tool/ccpp-tool.pro
ccpp-tool.depends = ccpp-lib

ccpp-lib-tests.file = $$PWD/ccpp-tests/ccpp-tests.pro
ccpp-lib-tests.depends = ccpp-lib
