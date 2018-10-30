#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

static const std::string OPENCV_WINDOW = "/camera/image_converted";
static bool GUI;

void imageCb(const sensor_msgs::ImageConstPtr& msg){
  if(!msg){
    ROS_INFO("no image received in image_subscriber. returning...%s", msg);
    return;
  }
  cv_bridge::CvImageConstPtr cv_ptr;
  try{
    cv_ptr = cv_bridge::toCvShare(msg); // non mutable
  }catch (cv_bridge::Exception& e){
    ROS_ERROR("ROS error receiving msg. Possibly could not convert '%s'. error: %s", msg->encoding.c_str(), e.what());
  }
  if(GUI){
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(1);
  }
}

int main(int argc, char **argv){
  if(argc < 2){
    std::cout << "Empty argv[1]. Please specify. 1 (GUI) 0 (no GUI)" << std::endl;
    return 1;
  }
  ros::init(argc, argv, "image_subscriber", ros::init_options::AnonymousName);
  //std::cout << "getName():" << ros::this_node::getName() << std::endl;
  GUI = atoi(argv[1]);
  ros::NodeHandle nh;
  if(GUI){
    cv::namedWindow(OPENCV_WINDOW);
    cv::startWindowThread();
  }
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("/camera/image_converted", 1, imageCb);
  ros::spin();
  if(GUI){
    cv::destroyWindow(OPENCV_WINDOW);
  }
  return 0;
}