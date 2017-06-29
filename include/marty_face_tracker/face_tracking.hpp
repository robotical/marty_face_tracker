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

  ros::Publisher face_centroid_;

public:
  FaceTracker(ros::NodeHandle& nh);
  ~FaceTracker();

private:
  std::string sub_name;

  std::string face_pub_name, eye_pub_name, smile_pub_name;

  std::string face_centroid_name;

  std::string face_cascade, eye_cascade, smile_cascade;

  cv::CascadeClassifier face_classifier, eye_classifier, smile_classifier;

  std::vector<cv::Rect> faces, eyes, smiles;

  cv_bridge::CvImagePtr face_image, eye_image, smile_image, face_region;
  cv::Mat grey_image;

  void loadParams();
  void loadClassifiers();
  void rosSetup();
  void imageCb(const sensor_msgs::ImageConstPtr &msg);
  void detect();
  void detectEyes(cv::Mat roi);
  void detectSmile(cv::Mat roi);

};
