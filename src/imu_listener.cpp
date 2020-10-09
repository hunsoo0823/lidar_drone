#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/PoseStamped.h"

void imu_Callback(const sensor_msgs::Imu::ConstPtr& msg)
{
    ROS_INFO("Imu Orientation x: [%f], y: [%f], z:[%f], w:[%f]", msg->orientation.x,msg->orientation.y,msg->orientation.z,msg->orientation.w);
}

void pose_Callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    ROS_INFO("Pose : [%f]", msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "imu_listener");
    ros::NodeHandle n;

    ros::Rate loop_rate(10);
    ros::Subscriber sub_imu = n.subscribe("mavros/imu/data", 100, imu_Callback);
    ros::Subscriber sub_pose = n.subscribe("mavros/local_position/pose", 100, pose_Callback);

    ros::spin();


    return 0;
}