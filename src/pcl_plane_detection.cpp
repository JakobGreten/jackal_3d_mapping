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
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <termios.h>

pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclPCZed(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclPCClassifiedZed(new pcl::PointCloud<pcl::PointXYZRGB>);
sensor_msgs::PointCloud2::Ptr pcClassifieZed(new sensor_msgs::PointCloud2);
//pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");

ros::Publisher publicador;
tf::StampedTransform transform;

float zedMaxZ = 2.3;

void zedCallback(sensor_msgs::PointCloud2 pcZed)
{
    pcl::fromROSMsg(pcZed, *pclPCZed);
    *pclPCClassifiedZed = *pclPCZed;

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.01);

    seg.setInputCloud(pclPCZed);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() == 0)
    {
        PCL_ERROR("Could not estimate a planar model for the given dataset.\n");
        //return (-1);
    }

    std::cerr << "Model coefficients: " << coefficients->values[0] << " "
              << coefficients->values[1] << " "
              << coefficients->values[2] << " "
              << coefficients->values[3] << std::endl;

    std::cerr << "Model inliers: " << inliers->indices.size() << std::endl;
    for (std::size_t i = 0; i < inliers->indices.size(); ++i)
    {
        // for (const int idx : inliers->indices)
        // {
        int idx = inliers->indices[i];
        std::cerr << idx << "    " << pclPCZed->points[idx].x << " "
                  << pclPCZed->points[idx].y << " "
                  << pclPCZed->points[idx].z << std::endl;
        pclPCClassifiedZed->points[idx].r = 255;
        pclPCClassifiedZed->points[idx].g = 0;
        pclPCClassifiedZed->points[idx].b = 0;

        // }
    }

    pcl::toROSMsg(*pclPCClassifiedZed, *pcClassifieZed);
    publicador.publish(pcClassifieZed);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "StairClassifierNode");
    ros::NodeHandle ns;
    ros::NodeHandle np;
    tf::TransformListener tf_listener;
    ROS_INFO_STREAM("PCL Version: " << PCL_VERSION);
    if (PCL_VERSION_COMPARE(>, 1, 7, 2))
    {
        ROS_INFO_STREAM("Different PCL Version detected. Recommended Version: 1.7.2");
    }

    ros::Subscriber zed_subscriptor = ns.subscribe("zed_points_filtered", 1, zedCallback);
    //ros::Subscriber zed_subscriptor = ns.subscribe("/zed_node/point_cloud/cloud_registered", 1, zedCallback);
    //ros::Subscriber zed_subscriptor = ns.subscribe("/zed_points", 10, zedCallback);

    publicador = ns.advertise<sensor_msgs::PointCloud2>("zed_points_classified", 10);

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
