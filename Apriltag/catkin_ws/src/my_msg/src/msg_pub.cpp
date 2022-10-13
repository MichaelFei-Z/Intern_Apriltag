#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include "my_msg/detection.h"

geometry_msgs::PoseWithCovarianceStamped makeTagPose(
    const Eigen::Matrix4d& transform,
    const Eigen::Quaternion<double> rot_quaternion,
    const std_msgs::Header& header)
{
  geometry_msgs::PoseWithCovarianceStamped pose;
  pose.header = header;
  //===== Position and orientation
  pose.pose.pose.position.x    = transform(0, 3);
  pose.pose.pose.position.y    = transform(1, 3);
  pose.pose.pose.position.z    = transform(2, 3);
  pose.pose.pose.orientation.x = rot_quaternion.x();
  pose.pose.pose.orientation.y = rot_quaternion.y();
  pose.pose.pose.orientation.z = rot_quaternion.z();
  pose.pose.pose.orientation.w = rot_quaternion.w();
  return pose;
}

int main(int argc,char **argv)
{
    ros::init(argc,argv,"PubMessage");//这个将会作为节点名

    ros::NodeHandle n;
    ros::Publisher pubSensorInfo=n.advertise<my_msg::detection>("/detectInfo",10);//这个将作为话题名
    ros::Rate loopRate(1);

    int count=1;
    while(ros::ok())
    {
        my_msg::detection msg;
        geometry_msgs::Pose pose_;
        msg.ids = {0};
        msg.size = {0.5};
        Eigen::Matrix3d rot = Eigen::Matrix3d::Identity();
        Eigen::Quaterniond q (rot);
        pose_.position.x = 1;
        pose_.position.y = 1;
        pose_.position.z = 1;
        pose_.orientation.x = q.x();
        pose_.orientation.y = q.y();
        pose_.orientation.z = q.z();
        pose_.orientation.w = q.w();
        msg.pose.pose.pose = pose_;

        pubSensorInfo.publish(msg);
        ROS_INFO("Publish sensor Info: x=%f y=%f z=%f q1=%f q2=%f q3=%f q4=%f",
            msg.pose.pose.pose.position.x,
            msg.pose.pose.pose.position.y,
            msg.pose.pose.pose.position.z,
            msg.pose.pose.pose.orientation.x,
            msg.pose.pose.pose.orientation.y,
            msg.pose.pose.pose.orientation.z,
            msg.pose.pose.pose.orientation.w
        );
        loopRate.sleep();
    }
}
