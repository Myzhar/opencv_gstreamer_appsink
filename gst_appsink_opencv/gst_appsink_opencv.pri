message("Added gst_appsink_opencv")

PATH = $$PWD

INCLUDEPATH += $$PATH/include

PUBLIC_HEADERS += $$INCLUDEPATH/gst_appsink_opencv.hpp
HEADERS += $$PUBLIC_HEADERS

SOURCES +=  $$PATH/src/gst_appsink_opencv.cpp

#-------------------------------------------------
win32{
    message("Windows 32 bit [win32]")
    GSTREAMER_PATH = C:\gstreamer\1.0\x86

    LIBS += \
    -lintl \
    -lws2_32 \
    -lole32 \
    -lwinmm \
    -lshlwapi \
    -lffi
}

#########################################################################
# Linux Desktop 64 bit
linux-g++-64{
    message("Linux Desktop 64 bit [linux-g++-64]")

    GSTREAMER_PATH = /usr

    INCLUDEPATH += $$GSTREAMER_PATH/lib/x86_64-linux-gnu/glib-2.0/include

    LIBS += \
        -lopencv_core \
        -lopencv_imgproc \
        -lopencv_highgui
}
#########################################################################

#########################################################################
# Linux
linux-g++{
    message("Linux Generic [linux-g++]")

    GSTREAMER_PATH = /usr

    #########################################################################
    # Jetson TX1/TX2
    #message("Jetson TX1/TX2")
    #INCLUDEPATH += \
    #    $$GSTREAMER_PATH/lib/aarch64-linux-gnu/glib-2.0/include \
    #    $$GSTREAMER_PATH/lib/aarch64-linux-gnu/gstreamer-1.0/include
    #########################################################################

    #########################################################################
    # ARM 32bit
    #message("ARM 32bit (Rpi3)")
    #INCLUDEPATH += $$GSTREAMER_PATH/lib/arm-linux-gnueabihf/glib-2.0/include
    #########################################################################

    LIBS += \
        -lopencv_core \
        -lopencv_imgproc \
        -lopencv_highgui
}

INCLUDEPATH += \
    $$GSTREAMER_PATH/include \
    $$GSTREAMER_PATH/include/gstreamer-1.0 \
    $$GSTREAMER_PATH/lib/gstreamer-1.0/include \
    $$GSTREAMER_PATH/include/glib-2.0 \
    $$GSTREAMER_PATH/lib/glib-2.0/include

LIBS += \
    -L$$GSTREAMER_PATH/lib \
    -lgio-2.0 \
    -lglib-2.0 \
    -lgobject-2.0 \
    -lgmodule-2.0 \
    -lgstreamer-1.0 \
    -lgstbase-1.0 \
    -lgstapp-1.0 \
    -lgstnet-1.0

#-------------------------------------------------
    

