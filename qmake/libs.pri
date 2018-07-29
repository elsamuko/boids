
# fltk
INCLUDEPATH += $$MAIN_DIR/libs/fltk/include

unix {
    QMAKE_CXXFLAGS += -isystem $$MAIN_DIR/libs/fltk/include
    QMAKE_CFLAGS += -isystem $$MAIN_DIR/libs/fltk/include
}

unix {
    LIBS += $$MAIN_DIR/libs/fltk/bin/$$PLATFORM/$$COMPILE_MODE/libfltk.a
    LIBS += $$MAIN_DIR/libs/fltk/bin/$$PLATFORM/$$COMPILE_MODE/libfltk_images.a
    # fltk-config --ldstaticflags
    LIBS += -lXrender -lXcursor -lXfixes -lXext -lXft -lfontconfig -lXinerama -lpthread -ldl -lm -lX11
}

macx {
    LIBS += -lc++ -lpthread -framework Cocoa
}

win32 {
    # fltk
    LIBS += /LIBPATH:$$MAIN_DIR/libs/fltk/bin/win/$$COMPILE_MODE
    LIBS += fltk$${COMPILE_FLAG}.lib
    LIBS += fltkimages$${COMPILE_FLAG}.lib
    LIBS += fltkjpeg$${COMPILE_FLAG}.lib
    LIBS += fltkpng$${COMPILE_FLAG}.lib

    # windows
    LIBS += DelayImp.lib
    LIBS += Gdi32.lib User32.lib Ole32.lib Advapi32.lib Shell32.lib Comdlg32.lib
    QMAKE_LFLAGS += /DELAYLOAD:Gdi32.dll
    QMAKE_LFLAGS += /DELAYLOAD:User32.dll
    QMAKE_LFLAGS += /DELAYLOAD:Ole32.dll
    QMAKE_LFLAGS += /DELAYLOAD:Advapi32.dll
    QMAKE_LFLAGS += /DELAYLOAD:Shell32.dll
    QMAKE_LFLAGS += /DELAYLOAD:Comdlg32.dll
    QMAKE_LFLAGS += /DELAYLOAD:Comctl32.dll
    message($$LIBS)
}
