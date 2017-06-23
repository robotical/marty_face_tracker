#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>

#include "opencv2/objdetect/objdetect.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/core/core.hpp"

#include <stdio.h>
#include <vector>


static const std::string FACE_DETECT_OUT = "Face detection";
static const std::string GREY = "BGR2GRAY";

// TODO: sort this mess out, can't have it hardcoded

static const std::string directory = "/home/helmi/catkin_ws/src/marty_face_tracker/classifiers/";
// static const std::string directory = "/classifiers/";

static const std::string face_cascade_default = directory + "haarcascade_frontalface_default.xml";
static const std::string face_cascade = directory + "haarcascade_face.xml";
static const std::string eye_cascade = directory + "haarcascade_eye.xml";
static const std::string smile_cascade = directory + "haarcascade_smile.xml";

static const std::string sub_name = "/marty/camera/image";
static const std::string face_pub_name = "/marty/face_tracking/image/faces";
static const std::string eye_pub_name = "/marty/face_tracking/image/eyes";
static const std::string smile_pub_name = "/marty/face_tracking/image/smiles";


class FaceTracker {
protected:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher face_pub_;
  image_transport::Publisher eye_pub_;
  image_transport::Publisher smile_pub_;

public:
  FaceTracker(ros::NodeHandle& nh);
  ~FaceTracker();
  void loadClassifiers();
  void imageCb(const sensor_msgs::ImageConstPtr &msg);

private:
  cv::CascadeClassifier face_classifier;
  cv::CascadeClassifier eye_classifier;
  cv::CascadeClassifier smile_classifier;


};
