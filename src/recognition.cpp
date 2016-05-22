#include "ros/ros.h"
#include "cv.h"
#include "highgui.h"
#include "cxcore.h"
#include <ml.h>
#include <stdio.h>
#include <iostream>
#include "ardrone_control/ROI.h"
#include "ardrone_control/ROINumber.h"
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include <sensor_msgs/image_encodings.h>

#define SQUARE_IMAGE_SIZE_R 16
#define SQUARE_IMAGE_SIZE_C 32

#define CV_RED cvScalar(255,0,0,0)
#define CV_GREEN cvScalar(0,255,0,0)
#define CV_WHITE cvScalar(255,255,255,0)

#define THRESHOLD 100

using namespace std;
using namespace cv;

CvSVM svm; 
Mat image1, image2, image3;
ardrone_control::ROINumber result;
const int rols = 1;
const int cols = SQUARE_IMAGE_SIZE_R*SQUARE_IMAGE_SIZE_C;
bool updated = false;
bool img_right = false;

Mat image_process(Mat img_white);

void  imageCallback(const ardrone_control::ROI &msg)
{

	if(msg.total>0)
	{
		cv_bridge::CvImagePtr cv_ptr1;
		cv_ptr1 = cv_bridge::toCvCopy(msg.image1, sensor_msgs::image_encodings::MONO8);

		threshold(cv_ptr1->image, image1, THRESHOLD, 255, THRESH_BINARY_INV);
        image1 = image_process(image1);

        if(img_right)
        {
        	imshow("img_get",image1);
			cvWaitKey(1);
	        
	        float training_data1[rols][cols];

	        for (int j=0; j<SQUARE_IMAGE_SIZE_R; j++)   //rols
	  		{  
		  		uchar* raw= image1.ptr<uchar>(j); 
	  	  		for (int i=0; i<SQUARE_IMAGE_SIZE_C; i++)  //cols
		  		{                   
			 		 training_data1[0][j*SQUARE_IMAGE_SIZE_R+i] = (float)raw[i];      				  
		  		}  
	  		}


	  		Mat training_data_mat1(rols, cols, CV_32FC1, training_data1);
	  		float response1 = svm.predict(training_data_mat1);
	  		training_data_mat1.release();
	  		cout<<"response1="<<response1<<endl;

	  		int response1_n = (int)response1;
	  		if(response1-response1_n < 0.05)
	  		{
	  			result.image1=response1_n;
	  		}
	  		else if(response1_n+1-response1 < 0.05)
	  		{
	  			result.image1=response1_n+1;
	  		}
	        else 
	    	{
	        	cout<<"Not Sure! "<<endl;
	        	result.image1 = 14;
	    	}
        }
        else result.image1 = 14;
			

	}

	if(msg.total>1)
	{
		cv_bridge::CvImagePtr cv_ptr2;
		cv_ptr2 = cv_bridge::toCvCopy(msg.image2, sensor_msgs::image_encodings::MONO8);

		threshold(cv_ptr2->image, image2, THRESHOLD, 255, THRESH_BINARY_INV);
		image2 = image_process(image2);

		if(img_right)
		{
			float training_data2[rols][cols];

	        for (int j=0; j<SQUARE_IMAGE_SIZE_R; j++)   //rols
	  		{  
		  		uchar* raw= image2.ptr<uchar>(j); 
	  	  		for (int i=0; i<SQUARE_IMAGE_SIZE_C; i++)  //cols
		  		{                   
			 		 training_data2[0][j*SQUARE_IMAGE_SIZE_R+i] = (float)raw[i];      				  
		  		}  
	  		}


	  		Mat training_data_mat2(rols, cols, CV_32FC1, training_data2);
	  		float response2 = svm.predict(training_data_mat2);
	  		training_data_mat2.release();
	  		cout<<"response2="<<response2<<endl;

	  		int response2_n = (int)response2;
	  		if(response2-response2_n < 0.05)
	  		{
	  			result.image2=response2_n;
	  		}
	  		else if(response2_n+1-response2 < 0.05)
	  		{
	  			result.image2=response2_n+1;
	  		}
	        else 
	    	{
	        	cout<<"Not Sure! "<<endl;
	        	result.image2 = 14;
	    	}
		}
		else result.image2 = 14;
                
		
	}


	if(msg.total>2)
	{
		cv_bridge::CvImagePtr cv_ptr3;
		cv_ptr3 = cv_bridge::toCvCopy(msg.image3, sensor_msgs::image_encodings::MONO8);

		threshold(cv_ptr3->image, image3, THRESHOLD, 255, THRESH_BINARY_INV);
		image3 = image_process(image3);

		if(img_right)
		{
			float training_data3[rols][cols];

	        for (int j=0; j<SQUARE_IMAGE_SIZE_R; j++)   //rols
	  		{  
		  		uchar* raw= image3.ptr<uchar>(j); 
	  	  		for (int i=0; i<SQUARE_IMAGE_SIZE_C; i++)  //cols
		  		{                   
			 		 training_data3[0][j*SQUARE_IMAGE_SIZE_R+i] = (float)raw[i];      				  
		  		}  
	  		}


	  		Mat training_data_mat3(rols, cols, CV_32FC1, training_data3);
	  		float response3 = svm.predict(training_data_mat3);
	  		training_data_mat3.release();
	  		cout<<"response3="<<response3<<endl;

	  		int response3_n = (int)response3;
	  		if(response3-response3_n < 0.05)
	  		{
	  			result.image3=response3_n;
	  		}
	  		else if(response3_n+1-response3 < 0.05)
	  		{
	  			result.image3=response3_n+1;
	  		}
	        else 
	    	{
	        	cout<<"Not Sure! "<<endl;
	        	result.image3 = 14;
	    	}	
		}
        
        
	}

    result.total = msg.total;
    result.pose1 = msg.pose1;
    result.pose2 = msg.pose2;
    result.pose3 = msg.pose3;
    updated = true;

	
}  

int main(int argc, char **argv)
{

  ros::init(argc, argv, "recognition");
  ros::NodeHandle nh;

  
  cout<<"SVM Vector Loading..."<<endl;
  svm.load( "/home/chg/SVM_DATA.xml" );
  cout<<"Finished!"<<endl;

  ros::Subscriber image_sub = nh.subscribe("/ROI_image", 1, imageCallback);
  ros::Publisher result_pub = nh.advertise<ardrone_control::ROINumber>("/number_result",5);  

  ros::Rate loop_rate(20);
  while (ros::ok())
  {
  	if(updated)
  	{
  		result_pub.publish(result);
  		updated = false;
  	}

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}


Mat image_process(Mat img_white)
{
	vector<vector<Point> > contours;

	Mat img_white_copy, img_white_copy2;
	img_white_copy = img_white.clone();
    img_white_copy2 = img_white.clone();

	Mat element = getStructuringElement( 0,Size( 10 ,10));
	// Threshold. There should only be one contour
	for(int i=0;;i++)
	{		
		// Find and draw contours     
		findContours(img_white_copy2, contours, RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
		cout<<"contours.size()="<<contours.size()<<endl;
		if(contours.size()==1) 
		{
			img_right = true;
			break;	
		}
		if(contours.size()==0||i==4) 
		{
			img_right = false;
			return img_white;
		}

		dilate(img_white, img_white, element);
		erode(img_white, img_white, element);		
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

	img_white.release();
	img_white_copy.release();
	img_white_copy2.release();

	return img_final;
}