#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <termios.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_datatypes.h>
#include <string>
#include <sstream>
// #include <zed_interfaces/set_pose.h>
// #include <zed_interfaces/set_poseRequest.h>
// #include <zed_interfaces/set_poseResponse.h>

ros::Publisher publicador;
tf::StampedTransform transform;

ros::Subscriber zed_pose_subscriptor;
ros::Subscriber optitrack_pose_subscriptor;

geometry_msgs::PoseStamped zed_pose;
geometry_msgs::PoseStamped optitrack_pose;

void startStaticTransformPublisher(){
    tf::Stamped<tf::Pose> zed_pose_tf;
    tf::poseStampedMsgToTF(zed_pose,zed_pose_tf);

    tf::Stamped<tf::Pose> optitrack_pose_tf;
    tf::poseStampedMsgToTF(optitrack_pose,optitrack_pose_tf);

    zed_pose_tf.inverseTimes(optitrack_pose_tf);

    geometry_msgs::PoseStamped pose_difference;
    tf::poseStampedTFToMsg(zed_pose_tf, pose_difference);

    tf2::Quaternion q_pose_difference,q_orig, q_rot, q_new;
    tf2::convert(pose_difference.pose.orientation , q_pose_difference);

    q_rot.setRPY(0, 3.14159265, 0);
    q_new = q_pose_difference*q_rot;
    q_new.normalize();
    
    // double x_p = pose_difference.pose.position.x;
    // double y_p = pose_difference.pose.position.y;
    // double z_p = pose_difference.pose.position.z;  
    double x_p = optitrack_pose.pose.position.x-zed_pose.pose.position.x;
    double y_p = optitrack_pose.pose.position.y-zed_pose.pose.position.y;
    double z_p = optitrack_pose.pose.position.z-zed_pose.pose.position.z;  
    double x_o = q_new.getX();
    double y_o = q_new.getY();
    double z_o = q_new.getZ();
    double w_o = q_new.getW();
    ROS_INFO_STREAM("publishing optitrack tf");

    std::ostringstream pose_string_stream;
    pose_string_stream<<"rosrun tf static_transform_publisher "<<x_p<<" "<<y_p<<" "<<z_p<<" "<<x_o<<" "<<y_o<<" "<<z_o<<" "<<w_o<<" /zed_map /optitrack_map 10 __name:=teeeest";
    std::string pose_string = pose_string_stream.str();
    // pose_string<<pose.pose.position.x<<" "<<pose.pose.position.y<<" "<<pose.pose.position.z<<" "<<q_rot.getX()<<" "<<q_rot.getY()<<" "q_rot.getZ();
    ROS_INFO_STREAM(pose_string);
    system(pose_string.c_str());
    //ros::shutdown();
    zed_pose_subscriptor.shutdown();
    optitrack_pose_subscriptor.shutdown();
}
void zedPoseCallback(geometry_msgs::PoseStamped pose_msg)
{
    zed_pose = pose_msg;
    // if(optitrack_pose.header.seq!=0){
    //     startStaticTransformPublisher();
    // }

    // //from tf_conversions.transformations import quaternion_from_euler
    // tf2::Quaternion q_orig, q_rot, q_new;
    // tf2::convert(pose.pose.orientation, q_orig);

    // q_rot.setRPY(1.570795, 0, 1.570795);
    // q_new = q_orig * q_rot;
    // q_new.normalize();

    // double x_p = pose.pose.position.x;
    // double y_p = pose.pose.position.y;
    // double z_p = pose.pose.position.z;
    // double x_o = q_new.getX();
    // double y_o = q_new.getY();
    // double z_o = q_new.getZ();
    // double w_o = q_new.getW();
    // ROS_INFO_STREAM(z_o);

    // ros::shutdown();
}
void optitrackPoseCallback(geometry_msgs::PoseStamped pose_msg)
{
    optitrack_pose = pose_msg;
    // if(zed_pose.header.seq!=0){
    //     startStaticTransformPublisher();
    // }
    
    tf2::Quaternion q_pose,q_orig, q_rot, q_new;
    tf2::convert(pose_msg.pose.orientation , q_pose);

    q_rot.setRPY(3.14159265, 3.14159265, 3.14159265);
    q_new = q_pose*q_rot;
    q_new.normalize();
    q_new = q_pose;
    double x_p = -pose_msg.pose.position.x;
    double y_p = -pose_msg.pose.position.y;
    double z_p = -pose_msg.pose.position.z;  
    
    double x_o = q_new.getX();
    double y_o = q_new.getY();
    double z_o = q_new.getZ();
    double w_o = q_new.getW();
    ROS_INFO_STREAM("publishing optitrack tf");

    std::ostringstream pose_string_stream;
    pose_string_stream<<"rosrun tf2_ros static_transform_publisher "<<x_p<<" "<<y_p<<" "<<z_p<<" "<<x_o<<" "<<y_o<<" "<<z_o<<" "<<w_o<<" /optitrack_map /zed_map __name:=teeeest";
    std::string pose_string = pose_string_stream.str();
    // pose_string<<pose.pose.position.x<<" "<<pose.pose.position.y<<" "<<pose.pose.position.z<<" "<<q_rot.getX()<<" "<<q_rot.getY()<<" "q_rot.getZ();
    ROS_INFO_STREAM(pose_string);
    system(pose_string.c_str());
    //ros::shutdown();
    zed_pose_subscriptor.shutdown();
    optitrack_pose_subscriptor.shutdown();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Transform_Zed_Loam_Node");
    ros::NodeHandle ns;
    ros::NodeHandle np;
    tf::TransformListener tf_listener;

    zed_pose_subscriptor = ns.subscribe("zed_node/pose", 1, zedPoseCallback);
    optitrack_pose_subscriptor = ns.subscribe("vrpn_client_node/jackal_tracker/pose", 1, optitrackPoseCallback);
    // ros::ServiceClient pose_service_client = ns.serviceClient<zed_interfaces::set_pose>("set_pose");
    

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
