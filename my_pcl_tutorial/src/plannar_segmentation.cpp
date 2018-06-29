#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/filters/extract_indices.h>

#include <pcl/common/centroid.h>

#include <geometry_msgs/PointStamped.h>

#include <pcl/pcl_config.h>

//  void printinfo()
// {
//   std::cout<<"OK"<<std::endl;
// #if PCL_VERSION_COMPARE(<, 1, 7, 2)
//     std::cout<<"ARG"<<std::endl;
// #endif
// };

ros::Publisher pub;
ros::Publisher pubc;

using namespace pcl;

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  // Create a container for the data.
  sensor_msgs::PointCloud2 output;

  //Conversions des donnée d'entrée en PCL
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*input,pcl_pc2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(pcl_pc2,*cloud);

  //Processing
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);

  seg.setInputCloud (cloud);
  seg.segment (*inliers, *coefficients);

  //ROS_INFO_STREAM("Coeff : "<<coefficients->values[0]<<" / "<<coefficients->values[1]<<" / "<<coefficients->values[2]<<" / "<<coefficients->values[3]);

  //Créations de l'output
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_output(new pcl::PointCloud<pcl::PointXYZ>);

  //Extraction du plan (inliners)
  // for (size_t i = 0; i < inliers->indices.size (); ++i)
  // {
  //   pcl_output->points.push_back( cloud->points[inliers->indices[i]]);

  // }

  //Extraction de l'objet (outliners) 
  //Attention bruit !
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*pcl_output);

  //Calcul du centre
  // CentroidPoint<pcl::PointXYZ> centroid;
  // for (size_t i = 0; i < pcl_output->size (); ++i)
  // {
  //   centroid.add(*pcl_output[i]);
  // }
  // pcl::PointXYZ c1;
  // centroid.get (c1);
  Eigen::Vector4f barycentre;
  pcl::compute3DCentroid(*pcl_output, barycentre);

  geometry_msgs::PointStamped center_msg;
  // for (size_t i = 0; i < pcl_output->size (); ++i)
  // {
  //   if( pcl_output->points[i].x)
  //   center.point.x += pcl_output->points[i]._PointXYZ::data[0];
  //   center.point.y += pcl_output->points[i]._PointXYZ::data[1];
  //   center.point.z += pcl_output->points[i]._PointXYZ::data[2];
  //   ROS_INFO_STREAM("Center clioyf : "<<pcl_output->points[i].x<<" / "<<pcl_output->points[i].y<<" / "<<pcl_output->points[i].z);
  // }
  // center.point.x = center.point.x/pcl_output->size ();
  // center.point.y = center.point.y/pcl_output->size ();
  // center.point.z = center.point.z/pcl_output->size ();
  center_msg.point.x = barycentre[0];
  center_msg.point.y = barycentre[1];
  center_msg.point.z = barycentre[2];
  center_msg.header.frame_id = input->header.frame_id;
  // ROS_INFO_STREAM("Center clioyf : "<<pcl_output->points[0]._PointXYZ::data[0]<<" / "<<pcl_output->points[0]._PointXYZ::data[1]<<" / "<<pcl_output->points[0]._PointXYZ::data[2]);
  // ROS_INFO_STREAM("barycentre : "<<barycentre(1,1)<<" / "<<barycentre(2,1)<<" / "<<barycentre(3,1)<<" / "<< barycentre(4,1));
  // ROS_INFO_STREAM("Center : "<<center_msg.point.x<<" / "<<center_msg.point.y<<" / "<<center_msg.point.z);

  //Envoie du nuage de points
  pcl::toPCLPointCloud2(*pcl_output,pcl_pc2);
  pcl_conversions::fromPCL(pcl_pc2, output);
  output.header.frame_id = input->header.frame_id;

  // Publish the data.
  pub.publish (output);
  pubc.publish( center_msg);
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/camera/depth_registered/points", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("/plane_output", 1);
  pubc = nh.advertise<geometry_msgs::PointStamped> ("/object_center", 1);

  // printinfo();
  // Spin
  ros::spin ();
}