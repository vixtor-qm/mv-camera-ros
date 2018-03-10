#include <iostream>
#include <opencv2/highgui.hpp>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <CameraApi.h>

#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <camera_info_manager/camera_info_manager.h>
/*
void imageDisparityCallback(const sensor_msgs::ImageConstPtr& msgDisp)
{
    try
    {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msgDisp, sensor_msgs::image_encodings::TYPE_8UC1);
        disparity =  (cv_ptr->image).clone();
        cv::imshow("disparity", disparity);
        cv::waitKey(10);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_INFO(" [%s]", msgDisp->encoding.c_str());
    }

}
*/
/*
void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& camInfo)
{
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            K.at<float>(i, j) = camInfo->K[i * 3 + j];
        }
    }

    float fx = camInfo->P[0];
    B  = -camInfo->P[3] / fx;
    f = fx;
}
*/

unsigned char * g_pRgbBuffer;

int main_mv(int argc, char **argv)
{
  int                     iCameraCounts = 1;
  int                     iStatus=-1;
  tSdkCameraDevInfo       tCameraEnumList;
  int                     hCamera;
  tSdkCameraCapbility     tCapability;      //设备描述信息
  tSdkFrameHead           sFrameInfo;
  BYTE*                   pbyBuffer;
  int                     iDisplayFrames = 10000;
  IplImage *              iplImage = NULL;
  int                     channel=3;

  // 0 for English, 1 for Chinese
  int status = CameraSdkInit(1);
  std::cout << "Camera SDK init status : " << status << std::endl;


  status = CameraEnumerateDevice(&tCameraEnumList,&iCameraCounts);
  if(iCameraCounts==0)
  {
    std::cout << "No camera found" << std::endl;
    std::cout << "Status = " << status << std::endl;
    return -1;
  }

  iStatus = CameraInit(&tCameraEnumList,-1,-1,&hCamera);

  if(iStatus!=CAMERA_STATUS_SUCCESS)
  {
    return -1;
  }

  std::cout << "Camera is detected" << std::endl;

  CameraGetCapability(hCamera,&tCapability);

  g_pRgbBuffer = (unsigned char*)malloc(tCapability.sResolutionRange.iHeightMax*tCapability.sResolutionRange.iWidthMax*3);

  CameraPlay(hCamera);


  if(tCapability.sIspCapacity.bMonoSensor)
  {
    channel=1;
    CameraSetIspOutFormat(hCamera,CAMERA_MEDIA_TYPE_MONO8);
  }else
  {
    channel=3;
    CameraSetIspOutFormat(hCamera,CAMERA_MEDIA_TYPE_BGR8);
  }


  ros::init(argc, argv, "mv-camera-node");
  ros::NodeHandle n;
  ros::Rate rate(30.0);
  while (n.ok())
  {
    if(CameraGetImageBuffer(hCamera,&sFrameInfo,&pbyBuffer,1000) == CAMERA_STATUS_SUCCESS)
    {
      CameraImageProcess(hCamera, pbyBuffer, g_pRgbBuffer, &sFrameInfo);

      cv::Mat image(sFrameInfo.iHeight, sFrameInfo.iWidth, CV_8UC3, g_pRgbBuffer, cv::Mat::AUTO_STEP);
      /*
      if (iplImage)
      {
        cvReleaseImageHeader(&iplImage);
      }
      iplImage = cvCreateImageHeader(cvSize(sFrameInfo.iWidth,sFrameInfo.iHeight),IPL_DEPTH_8U,channel);
      cvSetData(iplImage,g_pRgbBuffer,sFrameInfo.iWidth*channel);
      */
#if 0
      cvShowImage("OpenCV Demo",iplImage);
#else
      cv::imshow("OpenCV Demo", image);
#endif
      CameraReleaseImageBuffer(hCamera,pbyBuffer);

    }
    ros::spinOnce();
    rate.sleep();
  }

  CameraUnInit(hCamera);
  free(g_pRgbBuffer);
  return 0;
}
