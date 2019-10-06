BOOST_DIR=$$(BOOST_DIR)

isEmpty(BOOST_DIR): error("Please set BOOST_DIR in environment")

INCLUDEPATH += $$BOOST_DIR
INCLUDEPATH += $$PWD/../ccpplib
