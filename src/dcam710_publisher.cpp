#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/distortion_models.h>
#include <tf/transform_broadcaster.h>

// comment out strings if need not needed
#include <string>
#include <cerrno>
#include <iostream>
#include <stdint.h>
#include <ctype.h>
#include <vector>
#include <fstream>

#include "../dependencies/PicoZenseSDK_DCAM710/Include/PicoZense_api.h"

#define MAX_PATH_SIZE 1024

using namespace std;
using std::cout;
using std::cin;
using namespace cv;

sensor_msgs::ImagePtr imageToROSmsg(cv::Mat img, const std::string encodingType, std::string frameId, ros::Time t,int seqnum) {
  sensor_msgs::ImagePtr ptr = boost::make_shared<sensor_msgs::Image>();
  sensor_msgs::Image& imgMessage = *ptr;
  imgMessage.header.stamp = t;
  imgMessage.header.seq = seqnum;
  imgMessage.header.frame_id = frameId;
  imgMessage.height = img.rows;
  imgMessage.width = img.cols;
  imgMessage.encoding = encodingType;
  int num = 1; //for endianness detection
  imgMessage.is_bigendian = !(*(char *) &num == 1);
  imgMessage.step = img.cols * img.elemSize();
  size_t size = imgMessage.step * img.rows;
  imgMessage.data.resize(size);

  if (img.isContinuous())
    memcpy((char*) (&imgMessage.data[0]), img.data, size);
  else {
    uchar* opencvData = img.data;
    uchar* rosData = (uchar*) (&imgMessage.data[0]);
    for (unsigned int i = 0; i < img.rows; i++) {
      memcpy(rosData, opencvData, imgMessage.step);
      rosData += imgMessage.step;
      opencvData += img.step;
    }
  }
  return ptr;
}

void PicofillCamInfo(int _deviceIndex, sensor_msgs::CameraInfoPtr pico_cam_info_msg){

  pico_cam_info_msg->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

  PsFrameMode depthFrameMode;
  PsReturnStatus status = PsGetFrameMode(_deviceIndex, PsDepthFrame, &depthFrameMode);
  if(status == PsRetOK)
  {
    pico_cam_info_msg->width = depthFrameMode.resolutionWidth;
    pico_cam_info_msg->height = depthFrameMode.resolutionHeight;
  }

  PsCameraParameters params;
  status = PsGetCameraParameters(_deviceIndex, PsSensorType::PsDepthSensor, &params);

  pico_cam_info_msg->D.resize(5);
//  pico_cam_info_msg->D = {0.0};
  pico_cam_info_msg->D[0] = params.k1;   // k1
  pico_cam_info_msg->D[1] = params.k2;   // k2
  pico_cam_info_msg->D[2] = params.p1;   // k3
  pico_cam_info_msg->D[3] = params.p2;   // p1
  pico_cam_info_msg->D[4] = params.k3;   // p2
  //pico_cam_info_msg->D[5] = params.

  pico_cam_info_msg->K.fill(0.0);
  pico_cam_info_msg->K[0] = params.fx;
  pico_cam_info_msg->K[2] = params.cx;
  pico_cam_info_msg->K[4] = params.fy;
  pico_cam_info_msg->K[5] = params.cy;
  pico_cam_info_msg->K[8] = 1.0;

  pico_cam_info_msg->R.fill(0.0);

  for (int i = 0; i < 3; i++) {
    // identity
    pico_cam_info_msg->R[i + i * 3] = 1;
  }

  pico_cam_info_msg->P.fill(0.0);
  pico_cam_info_msg->P[0] = params.fx;
  pico_cam_info_msg->P[2] = params.cx;
  pico_cam_info_msg->P[5] = params.fy;
  pico_cam_info_msg->P[6] = params.cy;
  pico_cam_info_msg->P[10] = 1.0;

  pico_cam_info_msg->header.frame_id = "pico_depth_frame";

}

void publishCamInfo(sensor_msgs::CameraInfoPtr camInfoMsg, ros::Publisher pubCamInfo, ros::Time t) {
  static int seq = 0;
  camInfoMsg->header.stamp = t;
  camInfoMsg->header.seq = seq;
  pubCamInfo.publish(camInfoMsg);
  seq++;
}

int main(int argc, char** argv) {

  ros::init(argc, argv, "pico_dcam710_driver");
  ros::NodeHandle nh;

  //creating publisher for image through ImageTransport
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("pico/depth/image_raw", 1);

  /* Picozense API calls from this point onwards */
  bool colorMap = true;
  int32_t deviceIndex = 0;
  uint32_t slope = 3000;//4400;//1450;  // Near mode
  uint16_t threshold = 10;

  PsReturnStatus status = PsInitialize();
  ROS_INFO_STREAM( "Init status: " << status);

  if (status != PsReturnStatus::PsRetOK)
  {
      ROS_INFO_STREAM("Initialize failed!");
      system("pause");
      exit(0);
  }

  // publisher for camera information
  ros::Publisher pubPicoCamInfo = nh.advertise<sensor_msgs::CameraInfo>("pico/depth/camera_info", 1);

  int32_t deviceCount = 0;
  status = PsGetDeviceCount(&deviceCount);
  ROS_INFO_STREAM("Get device count status: " << status << " device count: " << deviceCount);

  status = PsSetThreshold(deviceIndex, threshold);
  ROS_INFO_STREAM("Set threshold status: " << status);

  //Set the Depth Range to Mid through PsSetDepthRange interface
  // PsNearRange, PsMidRange, PsFarRange
  status = PsSetDepthRange(deviceIndex, PsMidRange);
  ROS_INFO_STREAM( "Set depth range status: " << status);
  
//  status = PsSetDepthDistortionCorrectionEnabled(deviceIndex, false);
 
  // uint16_t pPulseCount = 4000;
  // status = PsSetPulseCount(deviceIndex, pPulseCount);
  // status = PsGetPulseCount(deviceIndex, p
 //  ROS_INFO_STREAM("Get pulse count: " << status << " pulse count: " << pPulseCount);

  status = PsOpenDevice(deviceIndex);

  if (status != PsReturnStatus::PsRetOK)
  {
    ROS_INFO_STREAM ("OpenDevice failed!");
    system("pause");
    exit(0);
  }
  // bool f_bFilter = false;
  // status = PsSetFilter(deviceIndex, PsSmoothingFilter, f_bFilter);

  int32_t depthRange = -1;
  status = PsGetDepthRange(deviceIndex, (PsDepthRange*)&depthRange);
  ROS_INFO_STREAM( "Get depth range status: " << status << " depth range: " << depthRange);

  status = PsStartFrame(deviceIndex, PsDepthFrame);
  ROS_INFO_STREAM( "Start Depth Frame status: " << status);

  // status = PsStartFrame(deviceIndex, PsIRFrame);
  // ROS_INFO_STREAM( "Start IR Frame status: " << status << "\n");

  PsFrameMode depthFrameMode;
  // PsFrameMode irFrameMode;

  status = PsGetFrameMode(deviceIndex, PsDepthFrame, &depthFrameMode);
  ROS_INFO_STREAM( "Get Depth Frame mode status: " << status);
  ROS_INFO_STREAM( "depthFrameMode.pixelFormat: " << depthFrameMode.pixelFormat);
  ROS_INFO_STREAM( "depthFrameMode.resolutionWidth: " << depthFrameMode.resolutionWidth);
  ROS_INFO_STREAM( "depthFrameMode.resolutionHeight: " << depthFrameMode.resolutionHeight);
  ROS_INFO_STREAM( "depthFrameMode.fps: " << depthFrameMode.fps << "\n");

  PsCameraParameters cameraParameters;
  status = PsGetCameraParameters(deviceIndex, PsDepthSensor, &cameraParameters);

  ROS_INFO_STREAM( "Get PsGetCameraParameters status: " << status);
  ROS_INFO_STREAM( "Camera Intinsic: ");
  ROS_INFO_STREAM( "Fx: " << cameraParameters.fx);
  ROS_INFO_STREAM( "Cx: " << cameraParameters.cx);
  ROS_INFO_STREAM( "Fy: " << cameraParameters.fy);
  ROS_INFO_STREAM( "Cy: " << cameraParameters.cy);
  ROS_INFO_STREAM( "Distortion Coefficient: ");
  ROS_INFO_STREAM( "K1: " << cameraParameters.k1);
  ROS_INFO_STREAM( "K2: " << cameraParameters.k2);
  ROS_INFO_STREAM( "K3: " << cameraParameters.k3);
  ROS_INFO_STREAM( "P1: " << cameraParameters.p1);
  ROS_INFO_STREAM( "P2: " << cameraParameters.p2);

  cv::Mat mDispImg;
  sensor_msgs::CameraInfoPtr pico_cam_info_msg_(new sensor_msgs::CameraInfo());

  static tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::Quaternion q;

  //creating a camera info message to be published
  PicofillCamInfo(deviceIndex, pico_cam_info_msg_);
  transform.setOrigin( tf::Vector3(0.02, -0.06, -0.015) );
  q.setRPY(0, 0, 0);
  transform.setRotation(q);

  /* transforms */
  ros::Rate loop_rate(1);
  int counter = 0;

  while (nh.ok()) {

    PsReadNextFrame(deviceIndex);

    PsFrame depthFrame;

    PsReturnStatus status = PsGetFrame(deviceIndex, PsDepthFrame, &depthFrame);

    if (status == PsRetOK) {
      mDispImg = cv::Mat(depthFrameMode.resolutionHeight, depthFrameMode.resolutionWidth, CV_16UC1, depthFrame.pFrameData);
    }

    ros::Time present_time = ros::Time::now();

    sensor_msgs::ImagePtr msg = imageToROSmsg(mDispImg, "16UC1", "pico_depth_frame" , present_time, counter);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "head_rgbd_sensor_rgb_frame", msg->header.frame_id));
    counter++;

    pub.publish(msg);

    publishCamInfo(pico_cam_info_msg_, pubPicoCamInfo, present_time);

    mDispImg.release();
    ros::spinOnce();
  }

  status = PsStopFrame(deviceIndex, PsDepthFrame);
  ROS_INFO_STREAM( "Stop Depth Frame status: " << status);
  status = PsCloseDevice(deviceIndex);
  ROS_INFO_STREAM( "CloseDevice status: " << status);
  status = PsShutdown();
  ROS_INFO_STREAM( "Shutdown status: " << status );

}
