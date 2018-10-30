#include "image_converter.h"

ImageConverter::ImageConverter(const std::string sub_topic, const std::string pub_topic)
: it_(nh_), img_in_(nullptr), img_out_(nullptr){
	sub_ = it_.subscribe(sub_topic, 1, &ImageConverter::imageCb, this);
	pub_ = it_.advertise(pub_topic, 1);
}

void ImageConverter::imageCb(const sensor_msgs::ImageConstPtr& msg){
	ROS_DEBUG("[1] imageCb called");
	img_in_ = msg;
}

void ImageConverter::process(){
	if(!img_in_){
	  	ROS_DEBUG("img_in_ is NULLPOINTER. returning..\n");
	  	return;
	}
	// Compare header metadata to not process the same image multiple times
	if(img_out_ && img_in_->header.stamp == img_out_->header.stamp){
		ROS_DEBUG("No new image!! returning..\n");
		return;
	}
	ROS_DEBUG("[2] processing...");

	// copy img_in_  so it's save to use with an ASyncSpinner (image won't change in the middle of processing)
	sensor_msgs::ImageConstPtr img_in = img_in_;
	cv_bridge::CvImagePtr cv_ptr;
	cv::Mat mat;
	try{
		cv_ptr = cv_bridge::toCvCopy(img_in); // mutable copy
	}catch(cv_bridge::Exception& e){
		ROS_ERROR("ROS error: %s", e.what());
		return;
	}
	mat = cv::Mat(cv::Size(MAX_IMAGE_WIDTH, MAX_IMAGE_HEIGHT), CV_8UC3, (cv_ptr->image).data);
	 for(int i=0; i<1; ++i){
		cv::cvtColor(mat, mat, CV_BGR2GRAY);
		cv::equalizeHist(mat, mat);
		//cv::cvtColor(mat, mat, CV_GRAY2BGR);
	}
	// convert output to ros msg and give it the input header metadata
	img_out_ = cv_bridge::CvImage(img_in->header, "mono8", mat).toImageMsg(); //bgr8
	ROS_DEBUG("[3] ...done processing!");
	publish();
}

void ImageConverter::publish(){
	ROS_DEBUG("[4] publish called\n");
	if(!img_out_){
		ROS_DEBUG("img_out_ is NULLPOINTER. Not publishing image. returning..\n");
		return;
	}
	pub_.publish(img_out_);
}

void ImageConverter::spin(){
	// this rate limits the loop overhead, especially when nothing new needs to be processed yet
	ros::Rate loop_rate(30); // [Hz]
	while(nh_.ok()){
		// do work only if there is a publisher where an image can be fetched from
		// and a subscriber the processed image can be send to
		if(pub_.getNumSubscribers() > 0 && sub_.getNumPublishers() > 0){
			ros::spinOnce();
			process();
		}
		loop_rate.sleep();
	}
}

int main(int argc, char** argv){
	ros::init(argc, argv, "image_converter");
	ROS_INFO("This is the ImageConverter");
	if(ros::console::set_logger_level(
		ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info)){
		ros::console::notifyLoggerLevelsChanged();
	}
	ImageConverter ic;
	ic.spin();
	return 0;
}