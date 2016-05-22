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

#define CLASS_SUM 10
#define SQUARE_IMAGE_SIZE 16

using namespace std;
using namespace cv;

CvSVM svm; 
Mat image1, image2, image3;
ardrone_control::ROINumber result;
const int rols = 1;
const int cols = SQUARE_IMAGE_SIZE*SQUARE_IMAGE_SIZE;
bool updated = false;

void  imageCallback(const ardrone_control::ROI &msg)
{

	if(msg.total>0)
	{
		cv_bridge::CvImagePtr cv_ptr1;
		cv_ptr1 = cv_bridge::toCvCopy(msg.image1, sensor_msgs::image_encodings::BGR8);

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

	if(msg.total>1)
	{
		cv_bridge::CvImagePtr cv_ptr2;
		cv_ptr2 = cv_bridge::toCvCopy(msg.image2, sensor_msgs::image_encodings::BGR8);

		threshold(cv_ptr2->image, image2, 100, 255, THRESH_BINARY_INV);
        
        float training_data2[rols][cols];

        for (int j=0; j<SQUARE_IMAGE_SIZE; j++)   //rols
  		{  
	  		uchar* raw= image2.ptr<uchar>(j); 
  	  		for (int i=0; i<SQUARE_IMAGE_SIZE; i++)  //cols
	  		{                   
		 		 training_data2[0][j*SQUARE_IMAGE_SIZE+i] = (float)raw[i];      				  
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
        	result.image1 = 14;
    	}
		
	}
	if(msg.total>2)
	{
		cv_bridge::CvImagePtr cv_ptr3;
		cv_ptr3 = cv_bridge::toCvCopy(msg.image3, sensor_msgs::image_encodings::BGR8);

		threshold(cv_ptr3->image, image3, 100, 255, THRESH_BINARY_INV);
        
        float training_data3[rols][cols];

        for (int j=0; j<SQUARE_IMAGE_SIZE; j++)   //rols
  		{  
	  		uchar* raw= image3.ptr<uchar>(j); 
  	  		for (int i=0; i<SQUARE_IMAGE_SIZE; i++)  //cols
	  		{                   
		 		 training_data3[0][j*SQUARE_IMAGE_SIZE+i] = (float)raw[i];      				  
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
        	result.image1 = 14;
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

  ros::Subscriber image_sub = nh.subscribe("/videofile/image_raw", 1, imageCallback);
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
