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


/*
 * Filter node for the OS1 LIDAR. Removes all points that are outside of a specified range on the Z axis.
 */

// Original point cloud
pcl::PointCloud<pcl::PointXYZI>::Ptr pclPCLidar(new pcl::PointCloud<pcl::PointXYZI>);

// Filtered point cloud as pcl::PointCloud
pcl::PointCloud<pcl::PointXYZI>::Ptr pclPCFilteredLidar(new pcl::PointCloud<pcl::PointXYZI>);

// Filtered point cloud as PointCloud2
sensor_msgs::PointCloud2::Ptr pcFilteredLidar(new sensor_msgs::PointCloud2);

ros::Publisher publisher;
tf::StampedTransform transform;

/**
 * Callback for the point cloud from the LIDAR. Publishes filtered point cloud.
 */
void lidarCallback(sensor_msgs::PointCloud2 pcLidar)
{
    //convert from PointCloud2 to pcl::PointCloud
    pcl::fromROSMsg(pcLidar, *pclPCLidar);
    
    // The CropBox filter is used to filter the point cloud.
    pcl::CropBox<pcl::PointXYZI> boxFilter;
    boxFilter.setInputCloud(pclPCLidar);

    // Only points that are outside of a thresholded range on the Z axis are filtered.
    float lidarMinX = -std::numeric_limits<float>::max();
    float lidarMinY = -std::numeric_limits<float>::max();;
    float lidarMinZ = -0.5;
    float lidarMaxX = std::numeric_limits<float>::max();
    float lidarMaxY = std::numeric_limits<float>::max();
    float lidarMaxZ = 3.0;

    boxFilter.setMin(Eigen::Vector4f(lidarMinX, lidarMinY, lidarMinZ, 1.0));
    boxFilter.setMax(Eigen::Vector4f(lidarMaxX, lidarMaxY, lidarMaxZ, 1.0));


    boxFilter.filter(*pclPCFilteredLidar);

    // convert back to PointCloud2
    pcl::toROSMsg(*pclPCFilteredLidar, *pcFilteredLidar);

    // publish filtered point cloud.
    publisher.publish(pcFilteredLidar);
}


/**
 * Main method gets called when starting the node.
 */
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

    // Subscribe to the topic the point cloud is published to.
    ros::Subscriber lidar_subscriber = ns.subscribe("os1_cloud_node/points", 1, lidarCallback);
    
    publisher = ns.advertise<sensor_msgs::PointCloud2>("lidar_points_filtered", 10);


   



    ros::Rate loop_rate(20);
    ros::spinOnce();

    // Main loop
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
               
    }
    ROS_INFO("Fin suscriptor");
    return 0;
}


