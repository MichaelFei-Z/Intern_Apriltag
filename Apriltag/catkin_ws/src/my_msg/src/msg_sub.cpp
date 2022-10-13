#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include "utils.hpp"
#include "my_msg/detection.h"
#include "apriltag_ros/AprilTagDetectionArray.h"
void printInfo(const apriltag_ros::AprilTagDetectionArray::ConstPtr & detectInfo)
{
    double tx, ty, tz, qx, qy, qz, qw;
    
    tx = detectInfo->detections[0].pose.pose.pose.position.x;
    ty = detectInfo->detections[0].pose.pose.pose.position.y;
    tz = detectInfo->detections[0].pose.pose.pose.position.z;

    qx = detectInfo->detections[0].pose.pose.pose.orientation.x;
    qy = detectInfo->detections[0].pose.pose.pose.orientation.y;
    qz = detectInfo->detections[0].pose.pose.pose.orientation.z;
    qw = detectInfo->detections[0].pose.pose.pose.orientation.w;

    Eigen::Quaterniond q (qx, qy, qz, qw);
    Eigen::Matrix3d rot = q.toRotationMatrix();
    Eigen::Vector3d euler = utils::rotationMatrixToEulerAngles(rot);

    ROS_INFO("Receieve detect info:\n x = %f\t y = %f\t z = %f\t r = %f\t p = %f\t y = %f\t \n\n", 
              tx, ty, tz, euler[0], euler[1], euler[2]);

}

void printReceieveSensorInfo(const my_msg::detection::ConstPtr & msg)
{
    double tx, ty, tz, qx, qy, qz, qw;

    tx = msg->pose.pose.pose.position.x;
    ty = msg->pose.pose.pose.position.y;
    tz = msg->pose.pose.pose.position.z;

    qx = msg->pose.pose.pose.orientation.x;
    qy = msg->pose.pose.pose.orientation.y;
    qz = msg->pose.pose.pose.orientation.z;
    qw = msg->pose.pose.pose.orientation.w;

    Eigen::Quaterniond q (qx, qy, qz, qw);
    Eigen::Matrix3d rot = q.toRotationMatrix();
    Eigen::Vector3d euler = utils::rotationMatrixToEulerAngles(rot);
    ROS_INFO("Receieve detect info:\n x = %f\t y = %f\t z = %f\t r = %f\t p = %f\t y = %f\t \n\n", 
                tx, ty, tz, euler[0], euler[1], euler[2]);

    // ROS_INFO("Receieve sensor info: ");
    // ROS_INFO("x = %f  y = %f  z = %f", 
    //           msg->pose.pose.pose.position.x, 
    //           msg->pose.pose.pose.position.y, 
    //           msg->pose.pose.pose.position.z);
    // ROS_INFO("r = %f  p = %f  y = %f", euler[0], euler[1], euler[2]);
}

int main(int argc,char **argv)
{
    ros::init(argc,argv,"GetDetectInfo");
    ros::NodeHandle n;
    ros::Subscriber sensorSub=n.subscribe("/detectInfo",10,printReceieveSensorInfo);

    ros::spin();
    return 0;
}
