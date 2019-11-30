MOC_DIR = .moc
UI_DIR = .ui
OBJECTS_DIR = .o
DESTDIR = $$OUT_PWD/../bin
equals(TEMPLATE, lib):DESTDIR = $$OUT_PWD/../lib
