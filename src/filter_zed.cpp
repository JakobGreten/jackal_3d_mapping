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
 * Filter node for the ZED camera. Removes all points that are outside of a specified range on the Z axis.
 */

// Original point cloud
pcl::PointCloud<pcl::PointXYZ>::Ptr pclPCZed(new pcl::PointCloud<pcl::PointXYZ>);

// Filtered point cloud as pcl::PointCloud
pcl::PointCloud<pcl::PointXYZ>::Ptr pclPCFilteredZed(new pcl::PointCloud<pcl::PointXYZ>);

// Filtered point cloud as PointCloud2
sensor_msgs::PointCloud2::Ptr pcFilteredZed(new sensor_msgs::PointCloud2);

ros::Publisher publisher;
tf::StampedTransform transform;


/**
 * Callback for the point cloud from the ZED camera. Publishes filtered point cloud.
 */
void zedCallback(sensor_msgs::PointCloud2 pcZed)
{
    //convert from PointCloud2 to pcl::PointCloud
    pcl::fromROSMsg(pcZed, *pclPCZed);
    
    
    // The CropBox filter is used to filter the point cloud.
    pcl::CropBox<pcl::PointXYZ> boxFilter;
    boxFilter.setInputCloud(pclPCZed);


    // Points that are outside of a thresholded range are filtered.
    float zedMinX = -std::numeric_limits<float>::max();
    float zedMinY = -std::numeric_limits<float>::max();;
    float zedMinZ = 0.0;
    float zedMaxX = std::numeric_limits<float>::max();
    float zedMaxY = 0.4;
    float zedMaxZ = 2.3;


    boxFilter.setMin(Eigen::Vector4f(zedMinX, zedMinY, zedMinZ, 1.0));
    boxFilter.setMax(Eigen::Vector4f(zedMaxX, zedMaxY, zedMaxZ, 1.0));

    boxFilter.filter(*pclPCFilteredZed);


    // convert back to PointCloud2
    pcl::toROSMsg(*pclPCFilteredZed, *pcFilteredZed);
    
    // publish filtered point cloud.
    publisher.publish(pcFilteredZed);
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
    ros::Subscriber zed_subscriber = ns.subscribe("zed_points_downsampled", 1, zedCallback);
    

    publisher = ns.advertise<sensor_msgs::PointCloud2>("zed_points_filtered", 10);



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


