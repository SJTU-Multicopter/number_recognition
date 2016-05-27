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

#define SQUARE_IMAGE_SIZE_R 24
#define SQUARE_IMAGE_SIZE_C 36

#define CV_RED cvScalar(255,0,0,0)
#define CV_GREEN cvScalar(0,255,0,0)
#define CV_WHITE cvScalar(255,255,255,0)

#define THRESHOLD 100

#define DETECT_NUM 10
#define RIGHT_NUM 8

using namespace std;
using namespace cv;

CvSVM svm; 
Mat image1, image2, image3, image_eg;
ardrone_control::ROINumber result;
const int rols = 1;
const int cols = SQUARE_IMAGE_SIZE_R*SQUARE_IMAGE_SIZE_C/4;;
bool updated = false;
bool img_right = false;

Mat image_process(Mat img_white);
float training_data_eg[cols];

int reg_counter1 = 0;
int reg_number1[10] = {0};
int reg_counter2 = 0;
int reg_number2[10] = {0};
int reg_counter3 = 0;
int reg_number3[10] = {0};

void  imageCallback(const ardrone_control::ROI &msg)
{

	if(msg.total>0)
	{
		cv_bridge::CvImagePtr cv_ptr1;
		cv_ptr1 = cv_bridge::toCvCopy(msg.image1, sensor_msgs::image_encodings::BGR8);
        
        image1 = cv_ptr1->image;
        image1 = image_process(image1);
        threshold(image1, image1, THRESHOLD, 255, THRESH_BINARY_INV);

        if(img_right)
        {
        	imshow("img_get",image1);
			cvWaitKey(1);
	        
	        float training_data1[cols];

	        int ptr_num = 0;
			for (int j=0; j<SQUARE_IMAGE_SIZE_R; j+=2)   //rols
			{  
				for (int i=0; i<SQUARE_IMAGE_SIZE_C; i+=2)  //cols
				{                   
					float temp = (image1.at<uchar>(i,j)+image1.at<uchar>(i+1,j)+image1.at<uchar>(i,j+1)+image1.at<uchar>(i+1,j+1))/4.0/255.0;
					training_data1[ptr_num] = temp;      				   
				    ptr_num ++;
				}  
			}


	  		Mat training_data_mat1(rols, cols, CV_32FC1, training_data1);
	  		float response1 = svm.predict(training_data_mat1);
	  		training_data_mat1.release();
	  		cout<<"response1="<<response1<<endl;

            reg_counter1 ++;
            int response1_int = (int)response1;
	  		reg_number1[response1_int] ++;

        }
        else result.image1 = 14;
			

	}

	if(msg.total>1)
	{
		cv_bridge::CvImagePtr cv_ptr2;
		cv_ptr2 = cv_bridge::toCvCopy(msg.image2, sensor_msgs::image_encodings::BGR8);
        
        image2 = cv_ptr2->image;
        image2 = image_process(image2);
        threshold(image2, image2, THRESHOLD, 255, THRESH_BINARY_INV);

        if(img_right)
        {
        	imshow("img_get",image2);
			cvWaitKey(1);
	        
	        float training_data2[cols];

	        int ptr_num = 0;
			for (int j=0; j<SQUARE_IMAGE_SIZE_R; j+=2)   //rols
			{  
				for (int i=0; i<SQUARE_IMAGE_SIZE_C; i+=2)  //cols
				{                   
					float temp = (image2.at<uchar>(i,j)+image2.at<uchar>(i+1,j)+image2.at<uchar>(i,j+1)+image2.at<uchar>(i+1,j+1))/4.0/255.0;
					training_data2[ptr_num] = temp;      				   
				    ptr_num ++;
				}  
			}


	  		Mat training_data_mat2(rols, cols, CV_32FC1, training_data2);
	  		float response2 = svm.predict(training_data_mat2);
	  		training_data_mat2.release();
	  		cout<<"response2="<<response2<<endl;

	  		reg_counter2 ++;
            int response2_int = (int)response2;
	  		reg_number2[response2_int] ++;

		}
		else result.image2 = 14;
                
		
	}


	if(msg.total>2)
	{
		cv_bridge::CvImagePtr cv_ptr3;
		cv_ptr3 = cv_bridge::toCvCopy(msg.image3, sensor_msgs::image_encodings::BGR8);
        
        image3 = cv_ptr3->image;
        image3 = image_process(image3);
        threshold(image3, image3, THRESHOLD, 255, THRESH_BINARY_INV);

        if(img_right)
        {
        	imshow("img_get",image3);
			cvWaitKey(1);
	        
	        float training_data3[cols];

	        int ptr_num = 0;
			for (int j=0; j<SQUARE_IMAGE_SIZE_R; j+=2)   //rols
			{  
				for (int i=0; i<SQUARE_IMAGE_SIZE_C; i+=2)  //cols
				{                   
					float temp = (image3.at<uchar>(i,j)+image3.at<uchar>(i+1,j)+image3.at<uchar>(i,j+1)+image3.at<uchar>(i+1,j+1))/4.0/255.0;
					training_data3[ptr_num] = temp;      				   
				    ptr_num ++;
				}  
			}


	  		Mat training_data_mat3(rols, cols, CV_32FC1, training_data3);
	  		float response3 = svm.predict(training_data_mat3);
	  		training_data_mat3.release();
	  		cout<<"response3="<<response3<<endl;

	  		reg_counter3 ++;
            int response3_int = (int)response3;
	  		reg_number3[response3_int] ++;
		}
        
        
	}
    
    //tell result
	switch(msg.total)
	{
		case 1:
        	if(reg_counter1>=DETECT_NUM)
        	{
        		bool found = false;
        		for(int i=0;i<10;i++)
        		{
        			if(reg_number1[i]>=RIGHT_NUM) {result.image1=i; found=true;}
        		}
        		if(found)
        		{
        			result.image2=10;
        			result.image3=10;
        			updated = true;
        		}
        		
        		reg_counter1 = 0;
        		memset(reg_number1,0,10*sizeof(int));
        	}
        	break;

		case 2:
			if(reg_counter1>=DETECT_NUM)
        	{
        		bool found = false;
        		for(int i=0;i<10;i++)
        		{
        			if(reg_number1[i]>=RIGHT_NUM) {result.image1=i; found=true;}
        		}
        		if(found)
        		{
        			result.image2=10;
        			result.image3=10;
        			updated = true;
        		}
        		
        		reg_counter1 = 0;
        		memset(reg_number1,0,10*sizeof(int));
        	}
        	if(reg_counter2>=DETECT_NUM)
        	{
        		bool found = false;
        		for(int i=0;i<10;i++)
        		{
        			if(reg_number2[i]>=RIGHT_NUM) {result.image2=i;found = true;}
        		}
        		if(found)
        		{
        			result.image3=10;
        			updated = true;
        		}
        		
        		reg_counter2 = 0;
        		memset(reg_number2,0,10*sizeof(int));
        	}
        	break;

		case 3:
			if(reg_counter1>=DETECT_NUM)
        	{
        		bool found = false;
        		for(int i=0;i<10;i++)
        		{
        			if(reg_number1[i]>=RIGHT_NUM) {result.image1=i;found = true;}
        		}
        		if(found)
        		{
        			result.image2=10;
        			result.image3=10;
        			updated = true;
        		}

        		reg_counter1 = 0;
        		memset(reg_number1,0,10*sizeof(int));
        	}
        	if(reg_counter2>=DETECT_NUM)
        	{
        		bool found = false;
        		for(int i=0;i<10;i++)
        		{
        			if(reg_number2[i]>=RIGHT_NUM) {result.image2=i;found = true;}
        		}
        		if(found)
        		{
        			result.image3=10;
        			updated = true;
        		}
        		
        		reg_counter2 = 0;
        		memset(reg_number2,0,10*sizeof(int));
        	}
        	if(reg_counter3>=DETECT_NUM)
        	{
        		bool found = false;
        		for(int i=0;i<10;i++)
        		{
        			if(reg_number3[i]>=RIGHT_NUM) {result.image3=i;found = true;}
        		}

        		if(found)updated = true;
        		
        		reg_counter3 = 0;
        		memset(reg_number3,0,10*sizeof(int));
        	}
        	break;

		default: break;

	}

    result.total = msg.total;
    result.pose1 = msg.pose1;
    result.pose2 = msg.pose2;
    result.pose3 = msg.pose3;

	
}  

int main(int argc, char **argv)
{

  ros::init(argc, argv, "recognition");
  ros::NodeHandle nh;

  
  cout<<"SVM Vector Loading..."<<endl;
  svm.load( "/home/chg/SVM_DATA.xml" );
  cout<<"Finished!"<<endl;

  ros::Subscriber image_sub = nh.subscribe("/ROI", 1, imageCallback);
  ros::Publisher result_pub = nh.advertise<ardrone_control::ROINumber>("/number_result",5);  

  
  /*eg*/
  /*
    char image_name[100] = "/home/chg/catkin_ws/src/number_recognition/images/test/3.png";
    image_eg = imread(image_name);
     
	image_eg = image_process(image_eg);
	threshold(image_eg, image_eg, THRESHOLD, 255, THRESH_BINARY_INV);

	imshow("img_eg",image_eg);
	cvWaitKey(100);
    //cout<<image_eg<<endl;

    int ptr_num = 0;
	for (int j=0; j<SQUARE_IMAGE_SIZE_R; j+=2)   //rols
	{  
		for (int i=0; i<SQUARE_IMAGE_SIZE_C; i+=2)  //cols
		{                   
			float temp = ((float)image_eg.at<uchar>(i,j)+(float)image_eg.at<uchar>(i+1,j)+(float)image_eg.at<uchar>(i,j+1)+(float)image_eg.at<uchar>(i+1,j+1))/4.0/255.0;      				   
		    training_data_eg[ptr_num] = temp;
		    ptr_num ++;
		}  
	}
	for(int i =0; i<cols; i++)   cout<<training_data_eg[i]<<" ";

	Mat training_data_mat_eg(rols, cols, CV_32FC1, training_data_eg);
	float response_eg = svm.predict(training_data_mat_eg);
	training_data_mat_eg.release();
	cout<<"response_eg="<<response_eg<<endl;
	*/

  cout<<"Ready to recognize!"<<endl;
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
    cvtColor(img_white, img_white, CV_BGR2GRAY);
    blur( img_white, img_white, Size(3,3) );  
	threshold(img_white, img_white, THRESHOLD, 255, THRESH_BINARY_INV);

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
		//cout<<"contours.size()="<<contours.size()<<endl;

		float img_x = img_white.cols;
		float img_y = img_white.rows;

		if(contours.size()==1 && fabs(img_x-img_y)<(img_x/10)) 
		{
			img_right = true;
			break;	
		}
		if(contours.size()==0||i==4||fabs(img_x-img_y)>=(img_x/10)) 
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