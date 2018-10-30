#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

static const std::string OPENCV_WINDOW = "/camera/image_raw";
static bool GUI;

int main(int argc, char** argv){
  if(argc < 2){
    std::cout << "Empty argv[1]. Please specify. 1 (GUI) 0 (no GUI)" << std::endl;
    return 1;
  }
  GUI = atoi(argv[1]);
  ros::init(argc, argv, "image_publisher");
  ROS_INFO("This is the image_publisher");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("/camera/image_raw", 1);
  cv::VideoCapture cap(0);
  if(!cap.isOpened())
    return 1;
  cv::Mat frame;
  sensor_msgs::ImagePtr msg;
  ros::Rate loop_rate(30); // [Hz]
  if(GUI){
    cv::namedWindow(OPENCV_WINDOW);
    cv::startWindowThread();
  }
  while (ros::master::check()) {
    cap >> frame;
    // Check if grabbed frame actually has content
    if(!frame.empty()) {
      std_msgs::Header header;
      header.stamp = ros::Time::now();
      msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
      pub.publish(msg);
      if(GUI){
        cv::imshow(OPENCV_WINDOW, frame);
        cv::waitKey(1);
      }
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  if (GUI){
    cv::destroyWindow(OPENCV_WINDOW);
  }
}