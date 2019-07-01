/*
**
**                           hsv_tracking.cpp
**
**
**********************************************************************/
/*
Last committed:     $Revision: 00
Last changed by:    $Author: KhueHM
Last changed date:  $Date:  $
ID:                 $Id:  $

**********************************************************************/

/*
* Include Header File
*/
#include <iostream>
#include <ctime>
#include <string>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "Socket.h"               // For UDPSocket and SocketException
#include <iostream>               // For cout and cerr
#include <cstdlib>                // For atoi()

using namespace std;
using namespace cv;


/*
* Definition
*/
//#define DEBUG

#define RASPI
#define CAM_INDEX					0


#ifdef RASPI
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>

#define CHANEL 				0
#define SPEED 				500000
#define	SS0					8       // GPIO 8

#define tx_Size				6
#define spi_enable			digitalWrite(SS0, 0)
#define spi_disable			digitalWrite(SS0, 1)


int spi_init(void);
void spi_send(uint8_t *data);

int spi_fd;
#endif // RASPI


/*
* Global variables and constants Declaration
*/
/* Create HSV value */
struct HSV_Val
{
	int H_max;
	int H_min;
	int S_max;
	int S_min;
	int V_max;
	int V_min;
};
struct HSV_Val hsv;

bool flag = true;

int dWidth;
int dHeight;
const uint8_t center_W = 50;
int object_area;

Point object;

uint8_t *spi_buffer;

uint16_t trash_X;

/*
* Functions Declaration
*/
void Create_HSV_Trackbar();
Mat pre_Process(Mat frameORG);
void draw_grid(Mat frameORG);
uint8_t *byte16_to_byte8(uint16_t byte16_X, uint16_t byte16_Y, uint16_t byte16_A);
uint16_t byte8_to_byte16(uint8_t * byte8);
void end_prog();

/*
* MAIN FUNCTION
*/
int main(int argc, char * argv[]) 
{
	/* MAIN SETUP */

	/* PROTOCOL CONFIGURATION - TRANSMITER SIDE */
    if ((argc < 3) || (argc > 3)) 
    { // Test for correct number of arguments
        cerr << "Usage: " << argv[0] << " <Server> <Server Port>\n";
        exit(1);
    }

    string servAddress = argv[1]; // First arg: server address
    unsigned short servPort = Socket::resolveService(argv[2], "udp");

	UDPSocket sock;
    int jpegqual =  ENCODE_QUALITY; // Compression Parameter

    Mat frame, send;
    vector < uchar > encoded;

#ifdef RASPI
	spi_init();
#endif // RASPI

	// Get video from CAM
	VideoCapture cap(CAM_INDEX);
	if (cap.isOpened() == false)
	{
		cout << "PLEASE CHECK THE CAMERA INDEX" << endl;
		cin.get();
		return -1;
	}

	dWidth = int(cap.get(CAP_PROP_FRAME_WIDTH));
	dHeight = int(cap.get(CAP_PROP_FRAME_HEIGHT));

	//cout << "Resolution of the video: " << dWidth << " x " << dHeight << endl;

#ifdef DEBUG
	// Create main WINDOW
	String main_window = "Cam FEED";
	namedWindow(main_window);
	String thresh_window = "Thesh Image";
	namedWindow(thresh_window);

	// Create HSV trackbar
	Create_HSV_Trackbar();

#endif // DEBUG

	Mat frameORG;
	Mat frameThresh;

	while (cap.isOpened())
	{				
		bool isSuccess = cap.read(frameORG);

		if (isSuccess == false)
		{
			cout << "Camera was disconnected" << endl;
			cin.get();
			break;
		}

		// Flip the frame
		flip(frameORG, frameORG, 1);
		frameThresh = pre_Process(frameORG);

		/* Stream video */
		if(frame.size().width==0)continue;//simple integrity check; skip erroneous data...
        resize(frame, send, Size(FRAME_WIDTH, FRAME_HEIGHT), 0, 0, INTER_LINEAR);
        vector < int > compression_params;
        compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
        compression_params.push_back(jpegqual);

        imencode(".jpg", send, encoded, compression_params);

        int total_pack = 1 + (encoded.size() - 1) / PACK_SIZE;

        int ibuf[1];
        ibuf[0] = total_pack;
        sock.sendTo(ibuf, sizeof(int), servAddress, servPort);

        for (int i = 0; i < total_pack; i++)
        sock.sendTo( & encoded[i * PACK_SIZE], PACK_SIZE, servAddress, servPort);

		/****************** MOMENTS *****************************
		Mat imgCircles = Mat::zeros(frameORG.size(), CV_8UC3);
		Moments oMoments = moments(frameThresh);

		double dM01 = oMoments.m01;
		double dM10 = oMoments.m10;
		double dArea = oMoments.m00;

		if (dArea > 10000)
		{
		//calculate the position of the ball
		int posX = dM10 / dArea;
		int posY = dM01 / dArea;

		if (posX >= 0 && posY >= 0)
		{
		//Draw a red circle encompassing object
		circle(imgCircles, Point(posX, posY), 25, Scalar(0, 0, 255), 1, 8, 1);
		circle(imgCircles, Point(posX, posY), 26, Scalar(0, 0, 255), 1, 8, 1);
		circle(imgCircles, Point(posX, posY), 27, Scalar(0, 0, 255), 1, 8, 1);
		circle(imgCircles, Point(posX, posY), 24, Scalar(0, 0, 255), 1, 8, 1);
		circle(imgCircles, Point(posX, posY), 23, Scalar(0, 0, 255), 1, 8, 1);
		}
		}

		line(frameORG, Point(320, 230), Point(320, 250), Scalar(255, 0, 255), 1, 8);
		line(frameORG, Point(310, 240), Point(330, 240), Scalar(255, 0, 255), 1, 8);

		frameORG = frameORG + imgCircles;
		**************************************************************/

		/********************* HOUGHCIRCLES ***************************
		vector<Vec3f> v3fCircles;
		HoughCircles(frameThresh, v3fCircles, CV_HOUGH_GRADIENT, 2, frameThresh.rows / 4, 100, 50, 10, 800);
		for (int i = 0; i < v3fCircles.size(); i++)
		{
		cout << "Position: " << v3fCircles[i][0] << " - " << v3fCircles[i][1] << "\n";
		circle(frameORG, Point((int)v3fCircles[i][0], (int)v3fCircles[i][1]), (int)v3fCircles[i][2], Scalar(0, 0, 255), 3);
		}
		***************************************************************/
		vector<vector<Point>> contours;
		findContours(frameThresh, contours, RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

		vector<double> areas(contours.size());
		for (int i = 0; i < (int)contours.size(); i++)
		{
			areas[i] = contourArea(Mat(contours[i]));
		}

		double max;
		Point maxPos;
		minMaxLoc(Mat(areas), 0, &max, 0, &maxPos);
		//drawContours(frameThresh, contours, maxPos.y, Scalar(255), CV_FILLED);

		Rect box;

		if (contours.size() >= 1)
		{
			box = boundingRect(contours[maxPos.y]);
			rectangle(frameORG, box.tl(), box.br(), Scalar(255, 0, 0), 2, 8, 0);
			

			object.x = box.x + box.width / 2;
			object.y = box.y + box.height / 2;
			circle(frameORG, object, 2, Scalar(255, 100, 0), 3);
			object_area = box.width * box.height;

			spi_buffer = byte16_to_byte8(object.x, object.y, object_area);
			

#ifdef RASPI
			spi_send(spi_buffer);
				

#else
			trash_X = byte8_to_byte16(spi_buffer);
			cout << "Pos: " << trash_X << "\n";
#endif // RASPI

		}
		/*
		else
		{
			spi_buffer = byte16_to_byte8(object.x, object.y, object_area);

#ifdef RASPI
			spi_send(spi_buffer);
#else
			trash_X = byte8_to_byte16(spi_buffer);
			cout << "Pos: " << trash_X << "\n";
#endif // RASPI
		}
		*/
		// Draw center grid
		draw_grid(frameORG);

#ifdef DEBUG
		// Show result frame
		imshow(main_window, frameORG);
		imshow(thresh_window, frameThresh);
#endif // DEBUG

		/* Press ESC to STOP */
		end_prog();
	}
	return EXIT_SUCCESS;
}

/*
* FUNCTIONS
*/
/* END function */
void Create_HSV_Trackbar()
{
	/* Create control WINDOW */
	const char* ctrl_window = "HSV CONTROL";
	namedWindow(ctrl_window, CV_WINDOW_AUTOSIZE);

	cvCreateTrackbar("Low H", ctrl_window, &hsv.H_min, 179);
	cvCreateTrackbar("High H", ctrl_window, &hsv.H_max, 179);
	cvCreateTrackbar("Low S", ctrl_window, &hsv.S_min, 255);
	cvCreateTrackbar("High S", ctrl_window, &hsv.S_max, 255);
	cvCreateTrackbar("Low V", ctrl_window, &hsv.V_min, 255);
	cvCreateTrackbar("High V", ctrl_window, &hsv.V_max, 255);
}

Mat pre_Process(Mat frameORG)
{
	Mat frame_out;
	Mat frameHSV;
	Mat frameThresh;
	// Convert BGR to HSV
	cvtColor(frameORG, frameHSV, COLOR_BGR2HSV);
	// Theshold the frame
#ifdef DEBUG
	inRange(frameHSV, Scalar(hsv.H_min, hsv.S_min, hsv.V_min), Scalar(hsv.H_max, hsv.S_max, hsv.V_max), frameThresh);
#else
	inRange(frameHSV, Scalar(0, 157, 128), Scalar(179, 255, 255), frameThresh);
#endif

	// Morphological Opening
	erode(frameThresh, frameThresh, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
	dilate(frameThresh, frameThresh, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

	// Morphological Closing
	dilate(frameThresh, frameThresh, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
	erode(frameThresh, frameThresh, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

	return frameThresh;
}


void end_prog()
{
	if (waitKey(10) == 27)
	{
		cout << "END PROGRAM" << endl;
		exit(EXIT_SUCCESS);
	}
}

void draw_grid(Mat frameORG)
{
	line(frameORG, Point(dWidth / 2, 0), Point(dWidth / 2, dHeight), Scalar(255, 100, 0), 1, 8);
	line(frameORG, Point(0, dHeight / 2), Point(dWidth, dHeight / 2), Scalar(255, 100, 0), 1, 8);
	line(frameORG, Point(dWidth / 2 - center_W, 0), Point(dWidth / 2 - center_W, dHeight), Scalar(0, 100, 0), 1, 8);
	line(frameORG, Point(dWidth / 2 + center_W, 0), Point(dWidth / 2 + center_W, dHeight), Scalar(0, 100, 0), 1, 8);
}


uint8_t *byte16_to_byte8(uint16_t byte16_X, uint16_t byte16_Y, uint16_t byte16_A)
{
	uint8_t * out_byte8;
	out_byte8[0] = (uint8_t)((byte16_X >> 8) & 0xFF); // BYTE HIGH
	out_byte8[1] = (uint8_t)(byte16_X & 0xFF);		  // BYTE LOW
	out_byte8[2] = (uint8_t)((byte16_Y >> 8) & 0xFF); // BYTE HIGH
	out_byte8[3] = (uint8_t)(byte16_Y & 0xFF);		  // BYTE LOW
	out_byte8[4] = (uint8_t)((byte16_A >> 8) & 0xFF); // BYTE HIGH
	out_byte8[5] = (uint8_t)(byte16_A & 0xFF);		  // BYTE LOW
	return out_byte8;
}

uint16_t byte8_to_byte16(uint8_t * byte8)
{
	uint16_t out_byte16;
	out_byte16 = byte8[0] << 8 | byte8[1];
	return out_byte16;
}

#ifdef RASPI

int spi_init(void)
{
	wiringPiSetupGpio();
	pinMode(SS0, OUTPUT);
	digitalWrite(SS0, 1);

	if ((wiringPiSPISetup(CHANEL, SPEED)) <0) {
		fprintf(stderr, "SPI Setup failed: %s\n", strerror(errno));
		return 0;
	}

	spi_fd = wiringPiSPIGetFd(0);
	
	return 1;
}

void spi_send(uint8_t *data)
{
	spi_enable;
	write(spi_fd, data, tx_Size);
	spi_disable;
}
#endif // RASPI