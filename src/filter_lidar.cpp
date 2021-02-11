#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/pcl_config.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
#include <termios.h>


pcl::PointCloud<pcl::PointXYZI>::Ptr pclPCLidar(new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr pclPCFilteredLidar(new pcl::PointCloud<pcl::PointXYZI>);
sensor_msgs::PointCloud2::Ptr pcFilteredLidar(new sensor_msgs::PointCloud2);
//pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");

ros::Publisher publicador;
tf::StampedTransform transform;


void lidarCallback(sensor_msgs::PointCloud2 pcLidar)
{
    pcl::fromROSMsg(pcLidar, *pclPCLidar);
    
    
    pcl::CropBox<pcl::PointXYZI> boxFilter;
    boxFilter.setInputCloud(pclPCLidar);

    float lidarMinX = -std::numeric_limits<float>::max();
    float lidarMinY = -std::numeric_limits<float>::max();;
    float lidarMinZ = -0.5;
    float lidarMaxX = std::numeric_limits<float>::max();
    float lidarMaxY = std::numeric_limits<float>::max();
    float lidarMaxZ = 3.0;

    boxFilter.setMin(Eigen::Vector4f(lidarMinX, lidarMinY, lidarMinZ, 1.0));
    boxFilter.setMax(Eigen::Vector4f(lidarMaxX, lidarMaxY, lidarMaxZ, 1.0));
    //pass.setFilterLimitsNegative (true);
    boxFilter.filter(*pclPCFilteredLidar);


    pcl::toROSMsg(*pclPCFilteredLidar, *pcFilteredLidar);
    publicador.publish(pcFilteredLidar);
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "FilterNode");
    ros::NodeHandle ns;
    ros::NodeHandle np;
    tf::TransformListener tf_listener;
    ROS_INFO_STREAM("PCL Version: " << PCL_VERSION);
    if (PCL_VERSION_COMPARE(>, 1, 7, 2))
    {
        ROS_INFO_STREAM("Different PCL Version detected. Recommended Version: 1.7.2");
    }

    ros::Subscriber lidar_subscriptor = ns.subscribe("os1_cloud_node/points", 1, lidarCallback);
    //ros::Subscriber zed_subscriptor = ns.subscribe("/zed_node/point_cloud/cloud_registered", 1, zedCallback);
    //ros::Subscriber zed_subscriptor = ns.subscribe("/zed_points", 10, zedCallback);

    publicador = ns.advertise<sensor_msgs::PointCloud2>("lidar_points_filtered", 10);


   




    ros::Rate loop_rate(20);
    ros::spinOnce();
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();

        
               
    }
    ROS_INFO("Fin suscriptor");
    return 0;
}


