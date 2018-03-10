#include <iostream>
#include <opencv2/highgui.hpp>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <CameraApi.h>
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

unsigned char           * g_pRgbBuffer;  

int main(int argc, char **argv)
{
     int                     iCameraCounts = 1;
    int                     iStatus=-1;
    tSdkCameraDevInfo       tCameraEnumList;
    int                     hCamera;
    tSdkCameraCapbility     tCapability;      //设备描述信息
    tSdkFrameHead           sFrameInfo;
    BYTE*                   pbyBuffer;
    int                     iDisplayFrames = 10000;
    IplImage *iplImage = NULL;

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

    ros::init(argc, argv, "mv-camera-node");
    ros::NodeHandle n;
    ros::Rate rate(30.0);
    while (n.ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
    //cv::waitKey(0);
    return 0;
}
