/*
 * ROS wrapper for the ASIFT_matcher object.
 * Track an object described in the references in a RGBD stream and publish it's center.
 * @author : antoine.harle@etu.upmc.Fr
 * @see : ASIFT_matcher.cpp/.hpp, asift_match.launch
 */

#include "ROS_matcher.hpp"

ROS_matcher::ROS_matcher(): _status(MATCHER_STATUS_WAITING_INIT)
{
	std::string center_topic, image_topic, pointcloud_topic, reference_txt_path;
	std::vector<std::string> reference_data_paths;

	//Load Param
	_nh.param<std::string>("tracked_object", tracked_object, "Object");
	_nh.param<int>("num_tilt", _num_tilt, 8);
	_nh.param<int>("std_dev_filter_coeff", _filter_coeff, 0);

	_nh.param<std::string>("object_center_topic", center_topic,"/ASIFT_matcher/object_center");
	_nh.param<std::string>("image_topic", image_topic,"/camera/rgb/image_raw");
	_nh.param<std::string>("pointcloud_topic", pointcloud_topic,"/camera/depth_registered/points");
	
	//Publisher
	_center_pub = _nh.advertise<rviz_interface::NamedPoint>(center_topic, 1);

	//Subscriber
	// info_sub = new message_filters::Subscriber<sensor_msgs::CameraInfo>(_nh, "camera/rgb/camera_info", 1);
	_image_sub = new message_filters::Subscriber<sensor_msgs::Image>(_nh, image_topic, 1);
	_pointcloud_sub= new message_filters::Subscriber<sensor_msgs::PointCloud2>(_nh, pointcloud_topic, 1);
	
	Timesync = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(2), *_image_sub, *_pointcloud_sub);
	Timesync-> registerCallback(boost::bind(&ROS_matcher::cameraCallback, this, _1, _2));

	matcher.showInfo(false);
	//Load References
	if(_nh.getParam("reference_txt_path", reference_txt_path))
	{
		if(matcher.loadReferences(reference_txt_path.c_str()))
			_status = MATCHER_STATUS_IDLE;
	}
	else if(_nh.getParam("reference_data", reference_data_paths))
	{
		for(unsigned int i=0; i<reference_data_paths.size(); i++)
			matcher.addReference(reference_data_paths[i].c_str(),_num_tilt);

		if(matcher.getNbRef()>0)
			_status = MATCHER_STATUS_IDLE;
	}
	else
	{
		ROS_WARN("No reference data to initialize matcher.");
	}

	if(_status == MATCHER_STATUS_IDLE)
		ROS_INFO("Matcher Ready ! (%d references from %s)",matcher.getNbRef(), reference_txt_path.c_str());
}

ROS_matcher::~ROS_matcher()
{
	// delete info_sub;
	delete _image_sub;
	delete _pointcloud_sub;
	// delete Timesync;
}

void ROS_matcher::cameraCallback(const sensor_msgs::Image::ConstPtr& image_msg, const sensor_msgs::PointCloud2::ConstPtr& pointcloud_msg)
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
			matcher.distFilter(_filter_coeff);

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
						center_msg.name = tracked_object;

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