#pragma once
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <ros/console.h>

#define MAX_IMAGE_WIDTH  	640
#define MAX_IMAGE_HEIGHT 	480

class ImageConverter{
	public:
		ImageConverter(const std::string = "/camera/image_raw", 
						const std::string = "/camera/image_converted");
		/*
		* runs this node
		* image processing + publishing at the specified loop rate
		*/
		void spin();

	private:
		ros::NodeHandle nh_;
		image_transport::ImageTransport it_;
		image_transport::Subscriber sub_;
		image_transport::Publisher pub_;
		sensor_msgs::ImageConstPtr img_in_;
		sensor_msgs::ImagePtr img_out_;

		/*
		* image callback. receives new image msg from subscribed topic during ros spin 
		* @param the new image
		*/
		void imageCb(const sensor_msgs::ImageConstPtr&);
		
		/*
		* processes the received image, then calls publish
		*/
		void process();

		/*
		*  publish converted img as ros msg
		*/
		void publish();
};