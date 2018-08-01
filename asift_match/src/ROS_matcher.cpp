#include "ROS_matcher.hpp"

ROS_matcher::ROS_matcher(): _num_tilt(1), _status(MATCHER_STATUS_WAITING_INIT)
{
	_center_pub = _nh.advertise<geometry_msgs::PointStamped>("/ROS_matcher/center", 10);

	message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub(_nh, "/camera/rgb/camera_info", 1);
	message_filters::Subscriber<sensor_msgs::Image> image_sub(_nh, "/camera/rgb/image_raw", 1);
	message_filters::Subscriber<sensor_msgs::PointCloud2> pointcloud_sub(_nh, "/camera/depth_registered/points", 1);
	message_filters::TimeSynchronizer<sensor_msgs::CameraInfo, sensor_msgs::Image, sensor_msgs::PointCloud2> sync(info_sub, image_sub, pointcloud_sub, 10);
	sync.registerCallback(boost::bind(&ROS_matcher::cameraCallback, this, _1, _2, _3));


	unsigned int nb_ref =2;
	std::string refData[] = {
      "book_training/train_image_000.png", 
      "book_training/train_image_001.png", 
      "book_training/train_image_002.png", 
      "book_training/train_image_003.png"};

    for(unsigned int i=0; i<nb_ref;i++)
	{
		matcher.addReference(refData[i].c_str(), _num_tilt);
	}
}

void ROS_matcher::cameraCallback(const sensor_msgs::CameraInfo::ConstPtr& info_msg, const sensor_msgs::Image::ConstPtr& image_msg, const sensor_msgs::PointCloud2::ConstPtr& pointcloud_msg)
{
	if(_status == MATCHER_STATUS_IDLE)
	{
		unsigned int width = image_msg->width, height = image_msg->height;
		std::vector<float> image(height*width);

		//Conversion en niveau de gris
		if(image_msg->encoding == "yuv422")
		{
			for(unsigned int i=0; i<height*width; i++)
			{
				image[i]=image_msg->data[3*i];
			}
		}
		else
		{
			ROS_WARN("Encoding doesn't correspond to yuv422");
			return;
		}

		int nb_match=0;
		nb_match = matcher.match(image, width, height, _num_tilt);

		ROS_INFO("Match : %d", nb_match);
	}
	else
	{
		ROS_INFO("Matcher not ready to process");
	}
}