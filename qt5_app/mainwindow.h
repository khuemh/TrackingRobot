#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include <QtCore>
#include <QDialog>
#include <iostream>
#include <cstdlib>
#include "opencv2/opencv.hpp"
#include "Socket.h"
#include "config.h"

using namespace cv;
using namespace std;

#define PORT                56666
#define BUF_LEN             65540

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:
    int cvStream(void);
    int dhtStream(void);
    void on_button_Go_clicked();
    void on_button_Pause_clicked();

private:
    Ui::MainWindow *ui;

    int goFlag = 0;




    /* STREAM CONFIG */
    unsigned short servPort = PORT;
    char buffer[BUF_LEN]; // Buffer for echo string
    int recvMsgSize; // Size of received message
    string sourceAddress; // Address of datagram source
    unsigned short sourcePort; // Port of datagram source

    /* DHT11 CONFIG */
    int dht11_dat[5];
    unsigned short dht_servPort = 56667;
    int dht_recvMsgSize;           // Size of received message
    string dht_sourceAddress;      // Address of datagram source
    unsigned short dht_sourcePort; // Port of datagram source

    QTimer * tmrTimer;
};

#endif // MAINWINDOW_H
