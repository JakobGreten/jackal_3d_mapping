#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <termios.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <string>
#include <sstream>


ros::Publisher publicador;
tf::StampedTransform transform;


void zedPoseCallback(geometry_msgs::PoseStamped pose)
{
    //from tf_conversions.transformations import quaternion_from_euler
    tf2::Quaternion q_orig, q_rot, q_new;
    tf2::convert(pose.pose.orientation , q_orig);

    q_rot.setRPY(1.570795, 0, 1.570795);
    q_new = q_orig*q_rot;
    q_new.normalize();
    
    double x_p = pose.pose.position.x;
    double y_p = pose.pose.position.y;
    double z_p = pose.pose.position.z;  
    double x_o = q_new.getX();
    double y_o = q_new.getY();
    double z_o = q_new.getZ();
    double w_o = q_new.getW();
    ROS_INFO_STREAM(z_o);

    std::ostringstream pose_string_stream;
    pose_string_stream<<"rosrun tf static_transform_publisher "<<x_p<<" "<<y_p<<" "<<z_p<<" "<<x_o<<" "<<y_o<<" "<<z_o<<" "<<w_o<<" /lego_map /camera_init 10 __name:=teeeest";
    std::string pose_string = pose_string_stream.str();
    // pose_string<<pose.pose.position.x<<" "<<pose.pose.position.y<<" "<<pose.pose.position.z<<" "<<q_rot.getX()<<" "<<q_rot.getY()<<" "q_rot.getZ();
    ROS_INFO_STREAM(pose_string);
    system(pose_string.c_str());
    ros::shutdown();
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "Transform_Zed_Loam_Node");
    ros::NodeHandle ns;
    ros::NodeHandle np;
    tf::TransformListener tf_listener;
    

    ros::Subscriber zed_pose_subscriptor = ns.subscribe("zed_node/pose", 1, zedPoseCallback);
    



   




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


