QT += core
QT -= gui

TARGET = TX_svm_v1
CONFIG += console
CONFIG -= app_bundle

TEMPLATE = app

SOURCES += main.cpp \
    detection/tagEdgeExtraction.cpp \
    detection/tophat.cpp \
    detection/dataAssociation.cpp \
    vibe/vibe.cpp \
    vibe/vibeDetect.cpp \
    codebook/codebook.cpp \
    kcf/fhog.cpp \
    kcf/kcftracker.cpp \
    kcf/tracker.cpp \
    svm/svm.cpp \
    socket.cpp

HEADERS += \
    detection/libyolo.h \
    detection/detection.h \
    detection/dataAssociation.h \
    vibe/vibe.h \
    codebook/codebook.h \
    kcf/recttools.hpp \
    kcf/fhog.hpp \
    kcf/labdata.hpp \
    kcf/ffttools.hpp \
    kcf/kcftracker.hpp \
    kcf/tracker.h \
    svm/svm.hpp \
    socket.h

INCLUDEPATH += /usr/local/include \
               /usr/local/include/opencv \
                /usr/local/include/opencv2
LIBS += /usr/local/lib/libopencv_highgui.so \
       /usr/local/lib/libopencv_core.so    \
       /usr/local/lib/libopencv_imgproc.so  \
        /usr/local/lib/libopencv_shape.so   \
        /usr/local/lib/libopencv_videoio.so \
        /usr/local/lib/libopencv_imgcodecs.so \
      /usr/local/lib/libopencv_ml.so  \
      /usr/local/lib/libopencv_objdetect.so
