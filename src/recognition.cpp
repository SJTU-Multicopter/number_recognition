#include "ros/ros.h"
#include "cv.h"
#include "highgui.h"
#include "cxcore.h"
#include <ml.h>
#include <stdio.h>
#include <iostream>

#define CLASS_SUM 10
#define SQUARE_IMAGE_SIZE 16

using namespace std;
using namespace cv;

CvSVM svm;    

int main(int argc, char **argv)
{

  ros::init(argc, argv, "recognition");
  ros::NodeHandle nh;
  
  cout<<"SVM Vector Loading..."<<endl;
  svm.load( "/home/chg/SVM_DATA.xml" );
  cout<<"Finished!"<<endl;
    

    const int rols = 1;
	const int cols = SQUARE_IMAGE_SIZE*SQUARE_IMAGE_SIZE;

	float training_data[rols][cols];

	char image_name[100] = "/home/chg/catkin_ws/src/number_recognition/images/processed/6/10.jpg";
   

	//read image
	Mat img_training = imread(image_name,CV_LOAD_IMAGE_GRAYSCALE);

	
	threshold(img_training, img_training, 150, 255, THRESH_BINARY_INV);
			
			//save data and label        

	for (int j=0; j<SQUARE_IMAGE_SIZE; j++)   //rols
	{  
		uchar* raw= img_training.ptr<uchar>(j); 
		for (int i=0; i<SQUARE_IMAGE_SIZE; i++)  //cols
		{                   
			training_data[0][j*SQUARE_IMAGE_SIZE+i] = (float)raw[i];      				  
		}  
	}

	img_training.release(); 

	Mat training_data_mat(rols, cols, CV_32FC1, training_data);
	float response = svm.predict(training_data_mat);

	cout<<"response="<<response<<endl;


  /*ros::Rate loop_rate(10);
  while (ros::ok())
  {

    ros::spinOnce();
    loop_rate.sleep();
  }*/

  return 0;
}
