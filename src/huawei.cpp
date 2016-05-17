#include "ros/ros.h"
#include "cv.h"
#include "highgui.h"
#include <iostream>

#define CV_RED cvScalar(255,0,0,0)
#define CV_GREEN cvScalar(0,255,0,0)
#define CV_WHITE cvScalar(255,255,255,0)

#define CLASS_SUM 10
#define SQUARE_IMAGE_SIZE 16

using namespace std;
using namespace cv;


int main(int argc, char **argv)
{

    ros::init(argc, argv, "huawei");

    ros::NodeHandle nh;

    cout<<"start reading"<<endl;


    char image_name[100] = "/home/chg/catkin_ws/src/number_recognition/images/raw/0/0.jpg";


    //read image
    Mat img_raw = imread(image_name,CV_LOAD_IMAGE_GRAYSCALE);

    if(img_raw.empty())
    {
        cout<<"no more found"<<endl;
        return -1;
    }

    threshold(img_raw, img_raw, 150, 255, THRESH_BINARY_INV);
    imshow("img_raw",img_raw);

    int step = 10;
    int num = (int)(img_raw.rows/step);
    int delt[num+1];
    int counter = 0;

    for(int i=0;i<img_raw.cols-step;i+=step)
    {
        int max_row=0;
        int min_row=10000;
        for(int j=0;j<img_raw.rows;j++)
        {
            if(img_raw.at<uchar>(j,i)>1)
            {
                if(j>max_row) max_row=j;
                if(j<min_row) min_row=j;
            }
        }

        delt[counter] = max_row - min_row;
        counter ++;
    }
    
    for(int i=0;i<counter;i++)
    {
        cout<<delt[i]<<" ";
    }
    cout<<endl;
    cvWaitKey(0);
    img_raw.release();


    return 0;
}
