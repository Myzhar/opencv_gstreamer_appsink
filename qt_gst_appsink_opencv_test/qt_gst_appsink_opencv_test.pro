QT += core
QT -= gui

CONFIG += c++11

include(../gst_appsink_opencv/gst_appsink_opencv.pri)

TARGET = qt_gst_appsink_opencv_test
CONFIG += console
CONFIG -= app_bundle

TEMPLATE = app

SOURCES += main.cpp
