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

#include <vector>
#include <string>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <visualization_msgs/Marker.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

ros::Publisher marker_pub;
tf::StampedTransform transform;
std::list<visualization_msgs::Marker> markers;
octomap::OcTree *tree;
int marker_id = 0;
void print_query_info(octomap::point3d query, octomap::OcTreeNode *node)
{
    if (node != NULL)
    {
        cout << "occupancy probability at " << query << ":\t " << node->getOccupancy() << endl;
    }
    // else
    //     cout << "occupancy probability at " << query << ":\t is unknown" << endl;
}
void visualizeVector(float x_pos, float y_pos, float z_pos, float roll, float pitch, float yaw)
{
    uint32_t shape = visualization_msgs::Marker::ARROW;
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = marker_id;
    marker_id++;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    q.normalize();

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = x_pos;
    marker.pose.position.y = y_pos;
    marker.pose.position.z = z_pos;
    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();
    marker.pose.orientation.w = q.w();

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.08;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();
    marker_pub.publish(marker);

    markers.push_back(marker);
}
void queryNormals(float x, float y, float z)
{
    octomap::OcTreeNode *result = NULL;
    octomap::point3d query = octomap::point3d(x, y, z);
    result = tree->search(query);

    std::vector<octomap::point3d> normals;
    if (tree->getNormals(query, normals))
    {
        print_query_info(query, result);

        for (unsigned i = 0; i < normals.size(); ++i)
        {
            //normals[i].rotate_IP(3.14159,3.14159,3.14159);
            cout << "\t" << normals[i].x() << "; " << normals[i].y() << "; " << normals[i].z() << endl;

            visualizeVector(query.x()+normals[i].x()*0.05, query.y()+normals[i].y()*0.05, query.z()+normals[i].z()*0.05, normals[i].roll(), normals[i].pitch(), normals[i].yaw());
            //visualizeVector(query.x(), query.y(), query.z(), normals[i].roll(), normals[i].pitch(), normals[i].yaw());
        }
    }
}

void pointCallback(geometry_msgs::PointStamped msg)
{
    ROS_INFO_STREAM("Clicked point. x: "<<msg.point.x<<" y: "<< msg.point.y<<" z: "<<msg.point.z);
    queryNormals(msg.point.x, msg.point.y, msg.point.z);
}


void octomapCallback(const octomap_msgs::Octomap::ConstPtr &msg)
{
    //loading octree from binary
    // const std::string filename = "/home/rrc/power_plant.bt";
    // octomap::OcTree temp_tree(0.1);
    // temp_tree.readBinary(filename);
    // fcl::OcTree* tree = new fcl::OcTree(std::shared_ptr<const octomap::OcTree>(&temp_tree));
    ROS_INFO("Octomap received");
    tree = dynamic_cast<octomap::OcTree *>(octomap_msgs::msgToMap(*msg));
    // octomap::point3d query;
    // octomap::OcTreeNode *result = NULL;

    for (float z = -0.5f; z < 1.995f; z += 0.04f)
    {
        for (float y = -2.58f; y < 2.58f; y += 0.04f)
        {
            for (float x = 1.08f; x < 2.727f; x += 0.04f)
            {
                // query = octomap::point3d(x, y, z);
                // result = tree->search(query);

                // std::vector<octomap::point3d> normals;
                // if (tree->getNormals(query, normals))
                // {
                //     print_query_info(query, result);

                //     for (unsigned i = 0; i < normals.size(); ++i)
                //     {
                //         cout << "\t" << normals[i].x() << "; " << normals[i].y() << "; " << normals[i].z() << endl;

                //         visualizeVector(query.x(), query.y(), query.z(), normals[i].roll(), normals[i].pitch(), normals[i].yaw());
                //     }
                // }
                queryNormals(x, y, z);
            }
        }
    }

    // query = octomap::point3d(1.51f, 0.379f, 0.166f);
    // result = tree->search(query);

    // std::vector<octomap::point3d> normals;
    // if (tree->getNormals(query, normals))
    // {

    //     cout << endl;
    //     std::string s_norm = (normals.size() > 1) ? " normals " : " normal ";
    //     cout << "MC algorithm gives " << normals.size() << s_norm << "in voxel at " << query << endl;
    //     for (unsigned i = 0; i < normals.size(); ++i)
    //     {
    //         cout << "\t" << normals[i].x() << "; " << normals[i].y() << "; " << normals[i].z() << endl;
    //         visualizeVector(query.x(), query.y(), query.z(), normals[i].roll(), normals[i].pitch(), normals[i].yaw());
    //     }
    // }
    // else
    // {
    //     cout << "query point unknown (no normals)\n";
    // }
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

    ros::Subscriber octomap_subscriptor = ns.subscribe("octomap_full", 1, octomapCallback);
    ros::Subscriber point_subscriptor = ns.subscribe("clicked_point", 1, pointCallback);

    marker_pub = ns.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    ros::Rate loop_rate(20);
    ros::spinOnce();
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
        // std::list<visualization_msgs::Marker>::iterator it = markers.begin();
        // while (it != markers.end())
        // {
        //     it->lifetime = ros::Duration();
        //     marker_pub.publish(*it);
        //     it++;
        // }
    }
    ROS_INFO("Fin suscriptor");
    return 0;
}
