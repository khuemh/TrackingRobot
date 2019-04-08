#-------------------------------------------------
#
# Project created by QtCreator 2019-04-07T07:19:10
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = qt_opencv_01
TEMPLATE = app

INCLUDEPATH += /usr/local/include/opencv
LIBS += -L/usr/local/lib -lopencv_core -lopencv_imgproc -lopencv_imgcodecs -lopencv_highgui -lopencv_videoio

SOURCES += main.cpp\
            mainwindow.cpp \
            Socket.cpp

HEADERS  += mainwindow.h \
            Socket.h \
            config.h

FORMS    += mainwindow.ui
