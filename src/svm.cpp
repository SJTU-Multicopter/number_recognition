#include "ros/ros.h"
#include "cv.h"
#include "highgui.h"
#include "cxcore.h"
#include <ml.h>
#include <stdio.h>

#define CLASS_SUM 10
#define SQUARE_IMAGE_SIZE 16

using namespace std;
using namespace cv;

int main(int argc, char **argv)
{

  ros::init(argc, argv, "svm");

  ros::NodeHandle nh;

  float label[6]={1,1,1,2,2,2};
  float traind[6][2]={{170,60},{175,65},{180,70},{155,45},{160,45},{157,50}};
  float testd[2]={174,65};

      CvMat matl,matd,mtestd;
      cvInitMatHeader(&matl,6,1,CV_32FC1,label);
      cvInitMatHeader(&matd,6,2,CV_32FC1,traind);
      cvInitMatHeader(&mtestd,1,2,CV_32FC1,testd);

      CvSVM svm;
      svm.train(&matd,&matl,NULL,NULL);
      float f = svm.predict(&mtestd);
      printf("predict f:  %d\n", int(f));

  ros::Rate loop_rate(10);
  while (ros::ok())
  {



    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
