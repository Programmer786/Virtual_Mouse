#include <opencv2/video/background_segm.hpp>
#include <opencv2\imgproc\imgproc.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <time.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <Windows.h>
#include <string.h>
#include <iostream>

#define K_CAMERA 0  //0 is define fisrt camra , 1 is 2nd camra and so on
using namespace cv;
using namespace std;

int const max_value = 255;
int const max_type = 4;
int thresh = 100;
int control_cursor = false;
int number_mean;

int data_count_finger[5];
int parser = 0;

char* window_name = "ControlAllValues";
String window_name2 = "Hand_HSV";

char* tracker_type = "Type: \n 0: Binary \n 1: Binary Inverted \n 2: Truncate \n 3: To Zero \n 4: To Zero Inverted";
char* tracker_value = "value";

time_t actual_click, prev_click;
int time_spent = 0;

void DetectCountour(Mat img);

double dist(Point x, Point y)
{
	return (x.x - y.x)*(x.x - y.x) + (x.y - y.y)*(x.y - y.y);
}

void LeftClickButton()
{
	INPUT Input = { 0 };
	//left down
	Input.type = INPUT_MOUSE;
	Input.mi.dwFlags = MOUSEEVENTF_LEFTDOWN;
	::SendInput(1, &Input, sizeof(INPUT));

	//left up
	::ZeroMemory(&Input, sizeof(INPUT));
	Input.type = INPUT_MOUSE;
	Input.mi.dwFlags = MOUSEEVENTF_LEFTUP;
	::SendInput(1, &Input, sizeof(INPUT));
}

void RightClickButton()
{
	
	INPUT Input = { 0 };
	//right down
	Input.type = INPUT_MOUSE;
	Input.mi.dwFlags = MOUSEEVENTF_RIGHTDOWN;
	::SendInput(1, &Input, sizeof(INPUT));

	//right up
	::ZeroMemory(&Input, sizeof(INPUT));
	Input.type = INPUT_MOUSE;
	Input.mi.dwFlags = MOUSEEVENTF_RIGHTUP;
	::SendInput(1, &Input, sizeof(INPUT));
}

void DetectCountour(Mat img)
{
	char text_to_show[100] = " ";
	int number = 0;
	int k;
	int ind_0, ind_1, ind_2;
	vector< vector<Point> > contours;
	vector<Vec4i> hierarchy;
	Mat drawing2 = Mat::zeros(img.size(), CV_8UC3);


	findContours(img, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

	if ((contours.size() > 0))
	{
		cout << "Contours = " << contours.size() << endl;

		//Approximate contours + vector to contain convex hull & defects points
		vector<vector<Point>> tcontours;
		vector<vector<int>> hull(contours.size());
		vector<vector<Vec4i>> convDef(contours.size());
		vector<vector<Point>> hull_points(contours.size());
		vector<vector<Point>> defect_points(contours.size());

		//Approximate contours to polygons + get circles
		vector<vector<Point>> contours_poly(contours.size());
		vector<Point2f> center(contours.size());
		vector<float> radius(contours.size()); 

		//Draw polygonal contours + circles
		Mat drawing = Mat::zeros(img.size(), CV_8UC3);

		//Draw contours
		for (int i = 0; i < contours.size(); i++)
		{
			//Defect contours that might correspond to a hand
			if (contourArea(contours[i]) >= 5000)
			{
				approxPolyDP(Mat(contours[i]), contours_poly[i], 3, true); //first check contours_poly if value 1 is avilable then value assign to contours
				minEnclosingCircle((Mat)contours_poly[i], center[i], radius[i]); //find center and draw radius from circle to center
				tcontours.push_back(contours[i]);

				//drawContours(drawing , tcontours ,-1 , cv::Scalar(255,255,255),2);

				//Defect Hull in current conture
				vector<vector<Point>> hulls(1);
				vector<vector<int>> hullsI(1);
				convexHull(Mat(tcontours[0]), hulls[0], false); //convexHull similar to contour approximation (convexityDefects = convexHull + Contoure)
				convexHull(Mat(tcontours[0]), hullsI[0], false);
				drawContours(drawing, hulls, -1, cv::Scalar(0, 255, 0), 2); //show in gream form contoure convexHull

				//Find Convex Defects
				vector<Vec4i> defects;

				//if there is a hull
				if(hullsI[0].size() > 0)
				{
					//Calls convexityDefects in order to detect a defect
					convexityDefects(tcontours[0], hullsI[0], defects);
					if (defects.size() >= 3)
					{
						//Display only circle whose redius is superior to 50 (this will me to have only the hand with)
						if (radius[i] > 50)
						{
							//draw the polygon(polygon with "n" sides)
							drawContours(drawing, contours_poly, i, Scalar(0, 0, 255), 1, 8, vector<Vec4i>(), 0, Point());

							//draw a circle around the hand
							cv::circle(drawing, center[i], (int)radius[i], Scalar(0, 0, 255), 2, 8, 0);

							//draw a circle in the center of the big circle surrounding the hand
							cv::circle(drawing, center[i], 5, Scalar(255, 0, 255), 2, 8, 0);

							//if the control is actived (by pressing spacebar)
							if (control_cursor == true)//true == 1
							{
								//Converting conter x/y position into mouse x/y position
								SetCursorPos((1366 / 400 * ((int)center[i].x - 100)), (800 / 200 * ((int)center[i].y - 200)));
							}
						}

						//Point to define the finger on the picture
						Point ptStart, ptEnd, ptFarthest;

						for (int j = 1; j < defects.size(); j++)
						{
							//Save defects data in points previously created
							int startidx = defects[j][0];
							ptStart = tcontours[0][startidx];
							int endidx = defects[j][1]; 
							ptEnd = tcontours[0][endidx];
							int faridx = defects[j][2];
							ptFarthest = tcontours[0][faridx];

							//display the defect if
							//the distance between ptStart & ptEnd might be long enough to consider that we have a finger
							//the defect is above the center
							//the circle around the hard is more that 60 which means at least there is one finger risen
							if ((dist(ptStart, ptFarthest) > 1500) && (ptStart.y < center[i].y) && (radius[i] >= 60))
							{
								//A Process start anticlock wise
								cv::circle(drawing, ptStart, 10, Scalar(255, 255, 255), 2, 8, 0);//draw circle fingure start point
								cv::circle(drawing, ptFarthest, 10, Scalar(0, 100, 255), 2, 8, 0);//draw circle Mid of two fingure (Start point & End Point)
								cv::line(drawing, ptEnd ,ptFarthest , Scalar(0, 255,255) ,2); //yalllow line
								//cv::line(drawing, ptStart ,ptEnd , Scalar(255,0,0) ,2);
								cv::line(drawing, ptStart, ptFarthest, Scalar(255, 100, 0), 2); // Blue line
								number++; //count risen finger
							}
						}
					}
				}
			}
		}

		//fill the array with the value of fingers counted
		data_count_finger[parser] = number;
		parser++;
		parser = parser % 5;

		//Do the mean
		number_mean = 0;
		for (int l = 0; l < 5; l++)
		{
			number_mean += data_count_finger[l];
		}
		number_mean /= 5;

		//checks if the spacebar was pressed in order to activate the mouse control
		if ((control_cursor == true)) //true == 1
		{
			if ((number_mean == 2) || (number_mean == 4))
			{

				//depending on the number of fingure detected
				switch (number_mean)
				{
				case 0:
					break;
				case 1:
					break;
				case 2:
					LeftClickButton();
					time(&prev_click);
					break;
				case 3:

					break;
				case 4:
					RightClickButton();
					time(&prev_click);
					break;
				case 5:
					break;
				default:
					break;
				}
			}
		}

		//Save the data information (number of finger and control mouse (1 = true ,0 = false)
		sprintf_s(text_to_show, 100, "Rauf Khan fingers:= %d, control:=%d", number_mean, control_cursor);

		//Print text_to_show on the screen
		cv::putText(drawing, text_to_show, Point(10, 50), FONT_HERSHEY_PLAIN, 2, Scalar(255, 255, 255));

		//Initialize number
		number = 0;
		//Show in a window
		cv::namedWindow("Contours", CV_WINDOW_AUTOSIZE);
		cv::imshow("Contours", drawing);
		cout << "+++++++++++++++++++++++++++++++++" << endl;
	}

}

int main(int argc, char** argv)
{
	VideoCapture cap(K_CAMERA); //capture the video from web cam

	if (!cap.isOpened())
	{
		cout << "Cannot open the web cam" << endl;
		return -1;
	}

	cv::namedWindow(window_name, WINDOW_AUTOSIZE); //create a window called "Control"

	cv::namedWindow("OriginalFream", WINDOW_NORMAL);

	//Default values for a a hand skin
	int iLowH = 100;  //100
	int iHighH = 122; //130
	
	int iLowS = 33; //73
	int iHighS = 151;

	int iLowV = 0; //93 //173
	int iHighV = 252;

	int blur_format = 13;

	int erode_value1 = 5;
	int erode_value2 = 5;
	int dilate_value1 = 5;
	int dilate_value2 = 5;

	//create trackbars in "COntrol" window
	cvCreateTrackbar("LowH", window_name, &iLowH, 179); //Hue (0 to 179)
	cvCreateTrackbar("HighH", window_name, &iHighH, 179);

	cvCreateTrackbar("LowS", window_name, &iLowS, 255); //Hue (0 to 255)
	cvCreateTrackbar("HighS", window_name, &iHighS, 255);

	cvCreateTrackbar("LowV", window_name, &iLowV, 255); //Hue (0 to 255)
	cvCreateTrackbar("HighV", window_name, &iHighV, 255);

	cvCreateTrackbar("blur", window_name, &blur_format, 15);

	cvCreateTrackbar("erode 1", window_name, &erode_value1, 10);

	cvCreateTrackbar("dilate 1", window_name, &dilate_value1, 10);

	cvCreateTrackbar("dilate 2", window_name, &dilate_value2, 10);

	cvCreateTrackbar("erode 2", window_name, &erode_value2, 10);

	cvCreateTrackbar("Threshold:", window_name, &thresh, 255);

	while (true)
	{
		Mat imgOriginal;

		bool bSuccess = cap.read(imgOriginal);  //read a new frame from video

		if (!bSuccess) //if not success ,break loop
		{
			cout << "Cannot read a frame rom video stream" << endl;
			break;
		}

	//Step#1>>>>>>>>>>>>>>
		imshow("OriginalFream", imgOriginal);  //show the original image

		// Init background substractor
		Ptr<BackgroundSubtractor> bg_model = createBackgroundSubtractorMOG2().dynamicCast<BackgroundSubtractor>();

		Mat imgHSV;

		flip(imgOriginal, imgOriginal, 1);  //Convert the captured frame from BGR to HSV

		cvtColor(imgOriginal, imgHSV, COLOR_RGB2HSV);  //Convert the captured frame from BGR to HSV

	//Step#2>>>>>>>>>>>>>>

		blur(imgHSV, imgHSV, Size(blur_format, blur_format));

		Mat imgThresholded;

		inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image

	//Step#3>>>>>>>>>>>>>>
		//imshow("inRange", imgThresholded); 

		//morphological opening (remove small object from the foreground)
		erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(erode_value1, erode_value1)));
		dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(dilate_value1, dilate_value1)));

		//morphological opening (fill small hole in the foreground)
		dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(dilate_value2, dilate_value2)));
		erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(erode_value2, erode_value2)));

	//Step#3>>>>>>>>>>>>>>
		cv::imshow("Thresholded Image", imgThresholded); //show the thresholded image

		DetectCountour(imgThresholded);

		if ((waitKey(15) == 32)) //wait for 'space' key press fro 15 mili secend. If 'space' key is pressed
		{
			//acticve the same control
			if (control_cursor == false)
			{
				control_cursor = true;
			}
			else {// deactivate the mouse control
				control_cursor = false;
			}
		}
	}
}