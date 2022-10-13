#include <sstream>
#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <visualization_msgs/Marker.h>
#include "print_info/utils.hpp"
#include "apriltag_ros/AprilTagDetectionArray.h"

class PrintInfo
{
public:
    PrintInfo()
    {
        detectSub = n.subscribe("/d415/color/tag_detections", 10, &PrintInfo::printDetectInfo, this);
        markerPub = n.advertise<visualization_msgs::Marker>("/marker", 10);

        marker.header.frame_id="/tag_0";
        marker.header.stamp = ros::Time::now();
        marker.ns = "basic_shapes";
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.id =0;
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

        marker.scale.z = 0.2;
        marker.color.b = 1.0;
        marker.color.g = 0.0;
        marker.color.r = 0.0;
        marker.color.a = 1.0;
    }
    void printDetectInfo(const apriltag_ros::AprilTagDetectionArray::ConstPtr & detectInfo)
    {
        if (detectInfo->detections.size() == 0)
            return;

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


        geometry_msgs::Pose pose;
        pose.position.x = tx + 0.1;
        pose.position.y = ty + 0.1;
        pose.position.z = tz + 0.1;

        Eigen::Vector3i info_trans, info_euler;
        info_trans << (int)(tx*1000), (int)(ty*1000), (int)(tz*1000);
        info_euler << (int)(euler[0] / M_PI * 180), (int)(euler[1] / M_PI * 180), (int)(euler[1] / M_PI * 180);
        std::ostringstream str;
        str << "[" 
            << info_trans[0] << "," << info_trans[1] << "," << info_trans[2] << "," 
            << info_euler[0] << "," << info_euler[1] << "," << info_euler[2] << "]";
        marker.text=str.str();
        // std::cout << marker.text << std::endl;
        marker.pose=pose;
        markerPub.publish(marker);
    }

private:
    visualization_msgs::Marker marker;
    ros::NodeHandle n; 
    ros::Publisher markerPub;
    ros::Subscriber detectSub;
};

 
int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "GetDetectInfo");
 
  //Create an object of class SubscribeAndPublish that will take care of everything
  PrintInfo print_info;
 
  ros::spin();
 
  return 0;
}
