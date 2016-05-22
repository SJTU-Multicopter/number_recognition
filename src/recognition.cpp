#include "ros/ros.h"
#include "cv.h"
#include "highgui.h"
#include "cxcore.h"
#include <ml.h>
#include <stdio.h>
#include <iostream>
#include "ardrone_control/ROI.h"
#include "sensor_msgs/Image.h"
#include <image_transport/image_transport.h>
#include "cv_bridge/cv_bridge.h"
#include <sensor_msgs/image_encodings.h>

#define CLASS_SUM 10
#define SQUARE_IMAGE_SIZE 16

using namespace std;
using namespace cv;

CvSVM svm; 
Mat image1, image2, image3;
const int rols = 1;
const int cols = SQUARE_IMAGE_SIZE*SQUARE_IMAGE_SIZE;

void  imageCallback(const ardrone_control::ROI &msg)
{
	
	if(msg.total>0)
	{
		cv_bridge::CvImagePtr cv_ptr1;
		sensor_msgs::Image image1_msg = msg.image1;
		cv_ptr1 = cv_bridge::toCvCopy(image1_msg, sensor_msgs::image_encodings::BGR8);
		//cv_ptr1->image;
		threshold(cv_ptr1->image, image1, 100, 255, THRESH_BINARY_INV);
        
        float training_data1[rols][cols];

        for (int j=0; j<SQUARE_IMAGE_SIZE; j++)   //rols
  		{  
	  		uchar* raw= image1.ptr<uchar>(j); 
  	  		for (int i=0; i<SQUARE_IMAGE_SIZE; i++)  //cols
	  		{                   
		 		 training_data1[0][j*SQUARE_IMAGE_SIZE+i] = (float)raw[i];      				  
	  		}  
  		}


  		Mat training_data_mat1(rols, cols, CV_32FC1, training_data1);
  		float response1 = svm.predict(training_data_mat1);
  		training_data_mat1.release();

  		cout<<"response1="<<response1<<endl;

	}

	if(msg.total>1)
	{
		cv_bridge::CvImagePtr cv_ptr2;
		//cv_ptr2 = cv_bridge::toCvCopy(msg.image2, sensor_msgs::image_encodings::MONO8);
		
	}
	if(msg.total>2)
	{
		cv_bridge::CvImagePtr cv_ptr3;
		//cv_ptr3 = cv_bridge::toCvCopy(msg.image3, sensor_msgs::image_encodings::MONO8);
		
	}


	
}  

int main(int argc, char **argv)
{

  ros::init(argc, argv, "recognition");
  ros::NodeHandle nh;

  
  cout<<"SVM Vector Loading..."<<endl;
  svm.load( "/home/chg/SVM_DATA.xml" );
  cout<<"Finished!"<<endl;

  ros::Subscriber image_sub = nh.subscribe("/videofile/image_raw", 1, imageCallback);
    

  ros::Rate loop_rate(10);
  while (ros::ok())
  {

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
