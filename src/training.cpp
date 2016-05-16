#include "ros/ros.h"
#include "cv.h"
#include "highgui.h"
#include "cxcore.h"
#include <ml.h>
#include <iostream>

#define CV_RED cvScalar(255,0,0,0)
#define CV_GREEN cvScalar(0,255,0,0)
#define CV_WHITE cvScalar(255,255,255,0)

using namespace std;
using namespace cv;


int main(int argc, char **argv)
{

  ros::init(argc, argv, "training");

  ros::NodeHandle nh;

  cout<<"start reading"<<endl;
  const char* imagename = "/home/chg/Downloads/IMG_20160516_211227.jpg";
 
  //read image
  Mat img_raw = imread(imagename);

  if(img_raw.empty())
  {
      cout<<"failed"<<endl;
      return -1;
  }
 
  //image process
  Mat img_gray, img_blur, img_white, img_white_copy, img_white_copy2;

  cvtColor(img_raw, img_gray, CV_BGR2GRAY);
  blur( img_gray, img_blur, Size(3,3) ); 

  Mat element = getStructuringElement( 0,Size( 5 ,5));

  threshold(img_blur, img_white, 100, 255, THRESH_BINARY_INV);

  vector<vector<Point> > contours;
  // Threshold. There should only be one contour
  for(int i=0;;i++)
  {
    dilate(img_white, img_white, element);
    erode(img_white, img_white, element);

    img_white_copy = img_white.clone();
    img_white_copy2 = img_white.clone();

    // Find and draw contours     
    findContours(img_white_copy2, contours, RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
    cout<<"contours.size()"<<contours.size()<<endl;
    if(contours.size()==1) break;
    if(i==4) 
    {
      cout<<"can not find contour of number!"<<endl;
      return -1;
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
  IplImage* img_final = cvCreateImage(cvSize(16,16), IPL_DEPTH_8U, 1);
  cvResize(&img_roi, img_final, CV_INTER_AREA);

  //show image
  imshow("img_gray", img_gray);
  //imshow("img_white", img_white);
  //imshow("img_white_copy",img_white_copy);
  //cvShowImage("img_roi",&img_roi);
  cvShowImage("img_final",img_final);

  cout<<"finished"<<endl;
  cvWaitKey(-1);
  
  //release
  img_raw.release();
  img_gray.release();
  img_blur.release();
  img_white.release();
  img_white_copy.release();
  img_white_copy2.release();

  /*ros::Rate loop_rate(10);
  while (ros::ok())
  {

    ros::spinOnce();
    loop_rate.sleep();
  }*/

  return 0;
}



  
