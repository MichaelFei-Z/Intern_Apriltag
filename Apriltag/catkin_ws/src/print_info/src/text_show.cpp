#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include<iostream>
using namespace std;
int main( int argc, char** argv )
{
    ros::init(argc, argv, "showline");
    ros::NodeHandle n;
    ros::Publisher markerPub = n.advertise<visualization_msgs::Marker>("TEXT_VIEW_FACING", 10);
    visualization_msgs::Marker marker;
    marker.header.frame_id="/map";
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

    ros::Rate r(1);
    double k= 1.0 / 300.0 ;
    int i = 0;
    while(i++ < 10.0)
    {
        geometry_msgs::Pose pose;
        pose.position.x =  i / 10.0;
        pose.position.y =  0;
        pose.position.z =0;
        ostringstream str;
        str<< k << "\n" << k;
        marker.text=str.str();
        marker.pose=pose;
        markerPub.publish(marker);
        cout<<"k="<<k<<endl;
        r.sleep();
    }
    return 0;
}