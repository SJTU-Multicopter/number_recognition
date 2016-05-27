#include "ros/ros.h"
#include "cv.h"
#include "highgui.h"
#include <iostream>

#define CV_RED cvScalar(255,0,0,0)
#define CV_GREEN cvScalar(0,255,0,0)
#define CV_WHITE cvScalar(255,255,255,0)

#define CLASS_SUM 11
#define SQUARE_IMAGE_SIZE_R 24
#define SQUARE_IMAGE_SIZE_C 36

#define THRESHOLD 60

using namespace std;
using namespace cv;


int main(int argc, char **argv)
{

	ros::init(argc, argv, "process");

	ros::NodeHandle nh;

	cout<<"start reading"<<endl;


	for(int class_seq=0; class_seq<CLASS_SUM; class_seq++)
	{
		for(int img_seq=0; ;img_seq++)
		{
			char image_name[100] = "/home/chg/catkin_ws/src/number_recognition/images/raw/";
			char save_path[150] = "/home/chg/catkin_ws/src/number_recognition/images/processed/";

			//add read path and name
			char class_num[4];
			sprintf(class_num,"%d/",class_seq);
			strcat(image_name,class_num);

			char img_num[8];
			sprintf(img_num,"%d.png",img_seq);
			strcat(image_name,img_num);
	  
			//add save path and name
			char img_num2[8];
			sprintf(img_num2,"%d.jpg",img_seq); 
			strcat(save_path,class_num);
			strcat(save_path,img_num2);


			//read image
			Mat img_raw = imread(image_name);

			if(img_raw.empty())
			{
				cout<<"Found "<<img_seq<<" images in Class "<<class_seq<<". Try next class."<<endl;
				break;
			}

			//image process
			Mat img_gray, img_blur, img_white, img_white_copy, img_white_copy2;

			cvtColor(img_raw, img_gray, CV_BGR2GRAY);
			blur( img_gray, img_blur, Size(3,3) ); 		

			threshold(img_blur, img_white, THRESHOLD, 255, THRESH_BINARY_INV);

			vector<vector<Point> > contours;

			Mat element = getStructuringElement( 0,Size( 10 ,10));
			// Threshold. There should only be one contour
			for(int i=0;;i++)
			{
				dilate(img_white, img_white, element);
				erode(img_white, img_white, element);

				img_white_copy = img_white.clone();
				img_white_copy2 = img_white.clone();

				// Find and draw contours     
				findContours(img_white_copy2, contours, RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
				cout<<"contours.size()="<<contours.size()<<endl;
				if(contours.size()==1) break;
				if(contours.size()==0 || i==4) 
				{
					cout<<"can not find contour of Class "<<class_seq<<", Image "<<img_seq<<"!"<<endl;
					break;
				}
			}

			drawContours(img_white_copy2, contours, 0, CV_RED, 2);

			// Find and draw bound rectangle
			vector<Rect> boundRect(contours.size());
			boundRect[0] = boundingRect(Mat(contours[0]));    
			rectangle(img_white_copy, boundRect[0].tl(), boundRect[0].br(), CV_WHITE, 0.2, 8, 0);

			// Get ROI area
			CvRect roi = CvRect(boundRect[0]);

			IplImage img_roi = img_white;
			cvSetImageROI(&img_roi,roi);
			 
			// Resize
			IplImage* img_final = cvCreateImage(cvSize(SQUARE_IMAGE_SIZE_R,SQUARE_IMAGE_SIZE_C), IPL_DEPTH_8U, 1);
			cvResize(&img_roi, img_final, CV_INTER_AREA);

			//show image
			//imshow("img_gray", img_gray);
			imshow("img_white", img_white);
			//imshow("img_white_copy",img_white_copy);
			//cvShowImage("img_roi",&img_roi);
			cvShowImage("img_final",img_final);
	  
			cvSaveImage(save_path, img_final);
			
			cvWaitKey(0);

			//release
			img_raw.release();
			img_gray.release();
			img_blur.release();
			img_white.release();
			img_white_copy.release();
			img_white_copy2.release();
			cvReleaseImage(&img_final);

		}
	}

  	cout<<"finished"<<endl;
	
	return 0;
}



	
