#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    namedWindow("Streaming", WINDOW_AUTOSIZE);

    tmrTimer = new QTimer(this);
    connect(tmrTimer, SIGNAL(timeout()), this, SLOT(cvStream()));
    connect(tmrTimer, SIGNAL(timeout()), this, SLOT(dhtStream()));
    tmrTimer->start(20);
}

MainWindow::~MainWindow()
{
    delete ui;
}

int MainWindow::cvStream(void)
{
    UDPSocket sock(servPort);

    try {
        do {
            recvMsgSize = sock.recvFrom(buffer, BUF_LEN, sourceAddress, sourcePort);
        } while (recvMsgSize > sizeof(int));
        int total_pack = ((int * ) buffer)[0];

        char * longbuf = new char[PACK_SIZE * total_pack];
        for (int i = 0; i < total_pack; i++)
        {
            recvMsgSize = sock.recvFrom(buffer, BUF_LEN, sourceAddress, sourcePort);
            if (recvMsgSize != PACK_SIZE)
            {
                cerr << "Received unexpected size pack:" << recvMsgSize << endl;
                continue;
            }
            memcpy( & longbuf[i * PACK_SIZE], buffer, PACK_SIZE);
        }

        Mat rawData = Mat(1, PACK_SIZE * total_pack, CV_8UC1, longbuf);


        Mat frame = imdecode(rawData, CV_LOAD_IMAGE_COLOR);

        if (frame.size > 0)
        {
            imshow("Streaming", frame);
        }

        free(longbuf);

    } catch (SocketException & e)
    {
        exit(1);
    }

    return 0;
}

int MainWindow::dhtStream(void)
{
    UDPSocket dht_sock(dht_servPort);

    try
    {
        dht_recvMsgSize = dht_sock.recvFrom(dht11_dat, sizeof(dht11_dat), dht_sourceAddress, dht_sourcePort);

        if ((dht11_dat[0] != 0))
        {
//            cout << "Humidity = " << dht11_dat[0] << "." << dht11_dat[1] << " % --- "
//                 << "Temperature = " << dht11_dat[2] << "." << dht11_dat[3] << " C"
//                 << endl;
            QString hum1 = QString::number(dht11_dat[0]);
            QString hum2 = QString::number(dht11_dat[1]);
            ui->label_humid_val->setText(QString("%1.%2").arg(hum1, hum2));

            QString temp1 = QString::number(dht11_dat[2]);
            QString temp2 = QString::number(dht11_dat[3]);
            ui->label_temp_val->setText(QString("%1.%2").arg(temp1, temp2));
        }
    }
    catch (SocketException & e)
    {
        exit(1);
    }
}

void MainWindow::on_button_Go_clicked()
{
    if(goFlag == 1)
    {
        ui->button_Go->setText("STOP");
        goFlag = 0;
    }
    else
    {
        ui->button_Go->setText("LET'S GO");
        goFlag = 1;
    }
}

void MainWindow::on_button_Pause_clicked()
{
    if(tmrTimer->isActive() == true)
    {
        tmrTimer->stop();
        ui->button_Pause->setText("RESUME");
    }
    else
    {
        tmrTimer->start(20);
        ui->button_Pause->setText("PAUSE");
    }
}
