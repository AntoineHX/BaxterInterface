#include "ROS_matcher.hpp"

ROS_matcher::ROS_matcher(): _num_tilt(8), _status(MATCHER_STATUS_IDLE)
{
	// _center_pub = _nh.advertise<geometry_msgs::PointStamped>("/ROS_matcher/center", 10);
	_center_pub = _nh.advertise<rviz_interface::NamedPoint>("/object_center", 1);

	info_sub = new message_filters::Subscriber<sensor_msgs::CameraInfo>(_nh, "/camera/rgb/camera_info", 1);
	image_sub = new message_filters::Subscriber<sensor_msgs::Image>(_nh, "/camera/rgb/image_raw", 1);
	pointcloud_sub= new message_filters::Subscriber<sensor_msgs::PointCloud2>(_nh, "/camera/depth_registered/points", 1);
	Timesync = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10),*info_sub, *image_sub, *pointcloud_sub);
	Timesync-> registerCallback(boost::bind(&ROS_matcher::cameraCallback, this, _1, _2, _3));

	// unsigned int nb_ref =2;
	// std::string refData[] = {
 //      "train_image_000.png", 
 //      "train_image_001.png"};

 //    for(unsigned int i=0; i<nb_ref;i++)
	// {
	// 	matcher.addReference(refData[i].c_str(), _num_tilt);
	// }
	ROS_INFO("Matcher Ready !");
}

ROS_matcher::~ROS_matcher()
{
	delete info_sub;
	delete image_sub;
	delete pointcloud_sub;
	// delete Timesync;
}

void ROS_matcher::cameraCallback(const sensor_msgs::CameraInfo::ConstPtr& info_msg, const sensor_msgs::Image::ConstPtr& image_msg, const sensor_msgs::PointCloud2::ConstPtr& pointcloud_msg)
{
	// ROS_INFO("Callback");

	if(_status == MATCHER_STATUS_IDLE)
	{
		unsigned int width = image_msg->width, height = image_msg->height;
		std::vector<float> image(height*width);

		// ROS_INFO("Image size : %d - %d", height, width);
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
		if(matcher.getNbRef()<1)
		{
			ROS_INFO("Building reference...");
			matcher.addReference(image, width, height, _num_tilt);
		}
		else
		{
		// ROS_INFO("Matching..."); 
		_status = MATCHER_STATUS_PROCESSING;

		ROS_INFO("Matching...");
		int nb_match=0;
		nb_match = matcher.match(image, width, height, _num_tilt);

		ROS_INFO("Match : %d", nb_match);

		//Recherche du point 3D
		if(nb_match>0)
		{
			//Publish 3D point
			int cx, cy;
			// geometry_msgs::PointStamped center_msg;
			rviz_interface::NamedPoint center_msg;
			matcher.distFilter(2);

			if(matcher.computeCenter(cx, cy))
			{
				//Conversions des donnée d'entrée en PCL
				pcl::PCLPointCloud2 pcl_pc2;
				pcl_conversions::toPCL(*pointcloud_msg,pcl_pc2);
				pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
				pcl::fromPCLPointCloud2(pcl_pc2,*cloud);

				// ROS_INFO("PointCloud Size : %d / %d", cloud->height, cloud->width);
				ROS_INFO("Center %f / %f",(float)cx/width,(float)cy/height);

				if(cloud->isOrganized())
				{
					pcl::PointXYZ center = cloud->at(cx,cy);


					if(!isnan(center.x) && !isnan(center.y) && !isnan(center.z))
					{
						center_msg.name = "6DOF";

						center_msg.header.frame_id = image_msg->header.frame_id;
						center_msg.point.x=center.x;
						center_msg.point.y=center.y;
						center_msg.point.z=center.z;
						_center_pub.publish(center_msg);
					}
					else
					{
						ROS_WARN("NaN Values");
					}
				}
				else
				{
					ROS_WARN("Pointcloud isn't organized");
				}
			}
			else
			{
				ROS_WARN("Failed to compute center");
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