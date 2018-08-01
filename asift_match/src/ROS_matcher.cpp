#include "ROS_matcher.hpp"

ROS_matcher::ROS_matcher(): _num_tilt(1), _status(MATCHER_STATUS_IDLE)
{
	_center_pub = _nh.advertise<geometry_msgs::PointStamped>("/ROS_matcher/center", 10);

	info_sub = new message_filters::Subscriber<sensor_msgs::CameraInfo>(_nh, "/camera/rgb/camera_info", 1);
	image_sub = new message_filters::Subscriber<sensor_msgs::Image>(_nh, "/camera/rgb/image_raw", 1);
	pointcloud_sub= new message_filters::Subscriber<sensor_msgs::PointCloud2>(_nh, "/camera/depth_registered/points", 1);
	Timesync = new message_filters::TimeSynchronizer<sensor_msgs::CameraInfo, sensor_msgs::Image>(*info_sub, *image_sub, 10);
	Timesync->registerCallback(boost::bind(&ROS_matcher::cameraCallback, this, _1, _2));

	// unsigned int nb_ref =2;
	// std::string refData[] = {
 //      "book_training/train_image_000.png", 
 //      "book_training/train_image_001.png"};

 //    for(unsigned int i=0; i<nb_ref;i++)
	// {
	// 	matcher.addReference(refData[i].c_str(), _num_tilt);
	// }
	ROS_INFO("Matcher Ready !");
}

ROS_matcher::~ROS_matcher()
{
	// delete info_sub;
	// delete image_sub;
	// delete pointcloud_sub;
	// delete Timesync;
}

void ROS_matcher::cameraCallback(const sensor_msgs::CameraInfo::ConstPtr& info_msg, const sensor_msgs::Image::ConstPtr& image_msg)
{
	ROS_INFO("Callback");

	if(_status == MATCHER_STATUS_IDLE)
	{
		unsigned int width = image_msg->width, height = image_msg->height;
		std::vector<float> image(height*width);

		// ROS_INFO("Size : %d - %d", height*width, height * image_msg->step);
		//Conversion en niveau de gris
		if(image_msg->encoding == "yuv422")
		{
			for(unsigned int i=0; i<height*width; i++)
			{
				image[i]=image_msg->data[2*i+1];
			}
		}
		else
		{
			ROS_WARN("Encoding doesn't correspond to yuv422");
			return;
		}

		//Matching
		if(matcher.getNbRef()<4)
		{
			matcher.addReference(image, width, height, _num_tilt);
		}
		else
		{
		ROS_INFO("Matching..."); 
		_status = MATCHER_STATUS_PROCESSING;

		int nb_match=0;
		nb_match = matcher.match(image, width, height, _num_tilt);

		ROS_INFO("Match : %d", nb_match);

		//Recherche du point 3D
		if(nb_match>0)
		{


			//Publish 3D point
			int cx, cy;
			geometry_msgs::PointStamped center_msg;
			//Provisoire
			if(matcher.computeCenter(cx, cy))
			{
				center_msg.header.frame_id = image_msg->header.frame_id;
				center_msg.point.x=cx;
				center_msg.point.y=cy;
				_center_pub.publish(center_msg);
			}
		}

		_status = MATCHER_STATUS_IDLE;
		}
	}
	else
	{
		ROS_INFO("Matcher not ready to process");
	}
}