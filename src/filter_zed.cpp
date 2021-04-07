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


pcl::PointCloud<pcl::PointXYZ>::Ptr pclPCZed(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr pclPCFilteredZed(new pcl::PointCloud<pcl::PointXYZ>);
sensor_msgs::PointCloud2::Ptr pcFilteredZed(new sensor_msgs::PointCloud2);
//pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");

ros::Publisher publicador;
tf::StampedTransform transform;

float zedMaxZ = 2.3;

void zedCallback(sensor_msgs::PointCloud2 pcZed)
{
    pcl::fromROSMsg(pcZed, *pclPCZed);
    
    
    pcl::CropBox<pcl::PointXYZ> boxFilter;
    boxFilter.setInputCloud(pclPCZed);

    float zedMinX = -std::numeric_limits<float>::max();
    float zedMinY = -std::numeric_limits<float>::max();;
    float zedMinZ = 0.0;
    float zedMaxX = std::numeric_limits<float>::max();
    float zedMaxY = 0.4;


    boxFilter.setMin(Eigen::Vector4f(zedMinX, zedMinY, zedMinZ, 1.0));
    boxFilter.setMax(Eigen::Vector4f(zedMaxX, zedMaxY, zedMaxZ, 1.0));
    //pass.setFilterLimitsNegative (true);
    boxFilter.filter(*pclPCFilteredZed);


    pcl::toROSMsg(*pclPCFilteredZed, *pcFilteredZed);
    publicador.publish(pcFilteredZed);
}

char getch()
{
    fd_set set;
    struct timeval timeout;
    int rv;
    char buff = 0;
    int len = 1;
    int filedesc = 0;
    FD_ZERO(&set);
    FD_SET(filedesc, &set);

    timeout.tv_sec = 0;
    timeout.tv_usec = 1000;

    rv = select(filedesc + 1, &set, NULL, NULL, &timeout);

    struct termios old = {0};
    if (tcgetattr(filedesc, &old) < 0)
        ROS_ERROR("tcsetattr()");
    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME] = 0;
    if (tcsetattr(filedesc, TCSANOW, &old) < 0)
        ROS_ERROR("tcsetattr ICANON");

    if(rv == -1)
        ROS_ERROR("select");
    // else if(rv == 0)
    //     ROS_INFO("no_key_pressed");
    else if(rv != 0)
        read(filedesc, &buff, len );

    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;
    if (tcsetattr(filedesc, TCSADRAIN, &old) < 0)
        ROS_ERROR ("tcsetattr ~ICANON");
    return (buff);
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

    ros::Subscriber zed_subscriptor = ns.subscribe("zed_points_downsampled", 1, zedCallback);
    //ros::Subscriber zed_subscriptor = ns.subscribe("/zed_node/point_cloud/cloud_registered", 1, zedCallback);
    //ros::Subscriber zed_subscriptor = ns.subscribe("/zed_points", 10, zedCallback);

    publicador = ns.advertise<sensor_msgs::PointCloud2>("zed_points_filtered", 10);


   




    ros::Rate loop_rate(20);
    ros::spinOnce();
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();

        // int c = getch(); // call your non-blocking input function
        // double diff_zed_max_range = 0.1;
        // if (c == 'w'){
        //     zedMaxZ+=diff_zed_max_range;
        //     ROS_INFO_STREAM("W pressed. Increasing Passthrough distance to: "<<zedMaxZ);
        // }
        // else if (c == 's'){
        //     zedMaxZ-=diff_zed_max_range;
        //     ROS_INFO_STREAM("S pressed. Decreasing Passthrough distance to: "<<zedMaxZ);

        // }
               
    }
    ROS_INFO("Fin suscriptor");
    return 0;
}


