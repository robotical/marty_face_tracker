/**
 * @file      face_tracking.hpp
 * @brief     Marty tracks faces, eyes and smiles
 * @author    Helmi Fraser <helmi@robotical.io>
 * @date      2017-07-03
 * @copyright (Apache) 2016 Robotical Ltd.
 */

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Int8MultiArray.h>
#include "std_msgs/MultiArrayDimension.h"
#include "marty_msgs/CentroidMsg.h"
#include "geometry_msgs/Point.h"

#include "opencv2/objdetect/objdetect.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/core/core.hpp"

#include <stdio.h>
#include <vector>

class FaceTracker {

protected:
  ros::NodeHandle nh_;

  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher face_pub_, eye_pub_, smile_pub_;

  ros::Publisher faces_centroid_pub_, smiles_centroid_pub_, eyes_centroid_pub_;

public:
  FaceTracker(ros::NodeHandle& nh);
  ~FaceTracker();

private:
  std::string sub_name;
  std::string face_pub_name, eye_pub_name, smile_pub_name;
  std::string faces_centroid_name, smiles_centroid_name, eyes_centroid_name;
  std::string face_cascade, eye_cascade, smile_cascade;
  std::vector<cv::Rect> faces, eyes, smiles;
  float detection_parameters[3][4];
  bool video_output, camera_detected;

  cv::CascadeClassifier face_classifier, eye_classifier, smile_classifier;
  cv_bridge::CvImagePtr face_image, eye_image, smile_image, face_region;
  cv::Mat grey_image;

  marty_msgs::CentroidMsg faces_centroid, smiles_centroid, eyes_centroid;

  void loadParams();
  void loadClassifiers();
  void rosSetup();
  void imageCb(const sensor_msgs::ImageConstPtr &msg);
  void publishData(bool image);
  void resetCentroidMsg(marty_msgs::CentroidMsg &msg, int size);
  void detectFaces(std::vector<cv::Rect> &facesVector, cv::Mat image);
  void detectEyes(std::vector<cv::Rect> &eyes_vector, cv::Mat roi);
  void detectSmile(std::vector<cv::Rect> &smiles_vector, cv::Mat roi);

};
