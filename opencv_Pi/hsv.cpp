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
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

/*
* Definition
*/
#define CAM_INDEX			0

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

/* Create HSV Trackbar function */
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

/*
* Functions Declaration
*/
void end_prog();

/*
MAIN FUNCTION
*/
int main()
{
	/* MAIN SETUP */
	// Get video from CAM
	VideoCapture cap(CAM_INDEX);
	if (cap.isOpened() == false)
	{
		cout << "ERROR" << endl;
		cin.get();
		return -1;
	}
	double dWidth = cap.get(CAP_PROP_FRAME_WIDTH);
	double dHeight = cap.get(CAP_PROP_FRAME_HEIGHT);

	//cout << "Resolution of the video: " << dWidth << " x " << dHeight << endl;

	// Create main WINDOW
	String main_window = "Cam FEED";
	namedWindow(main_window);
	String thresh_window = "Thesh Image";
	namedWindow(thresh_window);
	// Create HSV trackbar
	Create_HSV_Trackbar();

	while (cap.isOpened())
	{
		Mat frameORG;
		bool isSuccess = cap.read(frameORG);
		if (isSuccess == false)
		{
			cout << "Camera was disconnected" << endl;
			cin.get();
			break;
		}

		// Flip the frame
		flip(frameORG, frameORG, 1);

		// Convert BGR to HSV
		Mat frameHSV;
		cvtColor(frameORG, frameHSV, COLOR_BGR2HSV);

		// Theshold the frame
		Mat frameThresh;
		inRange(frameHSV, Scalar(hsv.H_min, hsv.S_min, hsv.V_min), Scalar(hsv.H_max, hsv.S_max, hsv.V_max), frameThresh);

		// Morphological Opening
		erode(frameThresh, frameThresh, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		dilate(frameThresh, frameThresh, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

		// Morphological Closing
		dilate(frameThresh, frameThresh, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		erode(frameThresh, frameThresh, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

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
		for (int i = 0; i < contours.size(); i++)
		{
			areas[i] = contourArea(Mat(contours[i]));
		}

		double max;
		Point maxPos;
		minMaxLoc(Mat(areas), 0, &max, 0, &maxPos);
		drawContours(frameThresh, contours, maxPos.y, Scalar(255), CV_FILLED);

		Point center;
		Rect box;

		if (contours.size() >= 1)
		{
			box = boundingRect(contours[maxPos.y]);
			rectangle(frameORG, box.tl(), box.br(), Scalar(255, 0, 0), 3, 8, 0);
		}
		center.x = box.x + box.width / 2;
		center.y = box.y + box.height / 2;

		cout << "Pos: " << center.x << " , " << center.y << "\n";
		// Show result frame
		imshow(main_window, frameORG);
		imshow(thresh_window, frameThresh);

		/* Press ESC to STOP */
		end_prog();
	}
	return EXIT_SUCCESS;
}

/*
* FUNCTIONS
*/
/* END function */
void end_prog()
{
	if (waitKey(10) == 27)
	{
		cout << "END PROGRAM" << endl;
		exit(EXIT_SUCCESS);
	}
}