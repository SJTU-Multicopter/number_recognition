#include "ros/ros.h"
#include "cv.h"
#include "highgui.h"
#include "cxcore.h"
#include <ml.h>
#include <iostream>
#include <stdio.h>

#define CLASS_SUM 11
#define SQUARE_IMAGE_SIZE_R 16
#define SQUARE_IMAGE_SIZE_C 24

using namespace std;
using namespace cv;

CvSVM svm_classifier;

int count_image();
void svm_train(int total_num);
float svm_classify(Mat realtime_img);

int main(int argc, char **argv)
{

	ros::init(argc, argv, "training");

	ros::NodeHandle nh;
	
	int total_num = count_image();
	cout<<"total_num="<<total_num<<endl;
	svm_train(total_num);
	
	return 0;
}


int count_image()
{
	/**counter**/
	int total_num = 0;

	for(int class_seq=0; class_seq<CLASS_SUM; class_seq++)
	{
		for(int img_seq=0; ;img_seq++)
		{
			char image_name[150] = "/home/chg/catkin_ws/src/number_recognition/images/processed/";

			//add read path and name
			char class_num[4];
			sprintf(class_num,"%d/",class_seq);
			strcat(image_name,class_num);

			char img_num[10];
			sprintf(img_num,"%d.jpg",img_seq);
			strcat(image_name,img_num);

			//read image
			Mat img_training = imread(image_name);

			if(img_training.empty())
			{
				//cout<<"Found "<<img_seq<<" images in Class "<<class_seq<<". Try next class."<<endl;
				break;
			}
			else total_num++;

		}
	}
	
	cout<<"Found "<<total_num<<" images!"<<endl;
	return total_num;
}


void svm_train(int total_num)
{
	/*Load Training Data*/
	int seq_counter = 0;
	//cout<<"total_num="<<total_num<<endl;

	const int rols = total_num;
	const int cols = SQUARE_IMAGE_SIZE_R*SQUARE_IMAGE_SIZE_C/4;
	//cout<<rols<<"  "<<cols<<endl;

	float training_data[rols][cols];
	float training_label[rols];

	for(int class_seq=0; class_seq<CLASS_SUM; class_seq++)
	{
		for(int img_seq=0; ;img_seq++)
		{
			char image_name[150] = "/home/chg/catkin_ws/src/number_recognition/images/processed/";

			//add read path and name
			char class_num[4];
			sprintf(class_num,"%d/",class_seq);
			strcat(image_name,class_num);

			char img_num[10];
			sprintf(img_num,"%d.jpg",img_seq);
			strcat(image_name,img_num);

			//read image
			Mat img_training = imread(image_name,CV_LOAD_IMAGE_GRAYSCALE);

			if(img_training.empty())
			{
				cout<<"Processed "<<img_seq<<" images in Class "<<class_seq<<". Start next class."<<endl;
				break;
			}
			else  
			{
				cout<<"Found "<<img_seq<<endl;
			}

			threshold(img_training, img_training, 150, 255, THRESH_BINARY_INV);
					
					//save data and label        
			training_label[seq_counter] = (float)class_seq;


      int ptr_num = 0;
			for (int j=0; j<SQUARE_IMAGE_SIZE_R; j+=2)   //rols
			{  
						for (int i=0; i<SQUARE_IMAGE_SIZE_C; i+=2)  //cols
						{                   
							training_data[seq_counter][ptr_num] = (img_training.at<uchar>(i,j)+img_training.at<uchar>(i+1,j)+img_training.at<uchar>(i,j+1)+img_training.at<uchar>(i+1,j+1))/4.0/255.0;      				  
						  cout<<training_data[seq_counter][ptr_num]<<"  ";
						  ptr_num ++;
						}  
			}
      
			img_training.release(); 

			seq_counter++;

			cout<<"******************"<<endl; 

		}
	}
	
	//cout<<training_data[total_num-1][SQUARE_IMAGE_SIZE*SQUARE_IMAGE_SIZE-1]<<endl;
	//cout<<training_label[2]<<endl;
	cout<<"Training..."<<endl;

	/*Start Training*/
	
	CvSVMParams params;
	params.svm_type    = CvSVM::C_SVC;
	params.kernel_type = CvSVM::LINEAR;
	params.term_crit   = cvTermCriteria(CV_TERMCRIT_ITER, 100, 1e-6);
	cout<<"Parameters Settled!"<<endl;

	Mat labels_mat(rols, 1, CV_32FC1, training_label);
	Mat training_data_mat(rols, cols, CV_32FC1, training_data);
	
	//cout<<labels_mat<<endl;
	//cout<<training_data_mat<<endl;

	svm_classifier.train(training_data_mat, labels_mat, Mat(), Mat(), params);
	svm_classifier.save( "/home/chg/SVM_DATA.xml" );
	cout<<"Training data saved in /home/chg/SVM_DATA.xml"<<endl;
	
	cout<<"Training finished!~"<<endl;
}
