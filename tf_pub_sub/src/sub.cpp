// #include <ros/ros.h>
// #include <tf/transform_listener.h>
// int main(int argc, char** argv){
//     ros::init(argc, argv, "demo_tf_listener");  //初始化一个监听器
 
//     ros::NodeHandle n; //创建一个句柄
 
//     tf::TransformListener listener; //声明一个监听器 
 
//     float saved_rx = 0.0;
//     float saved_ry = 0.0;
//     float saved_rz = 0.0;
 
//     ros::Rate rate(10.0);
//     while (ros::ok())
//     {
//         tf::StampedTransform transform;
 
//         try{
//             ros::Time now = ros::Time::now();
//             //订阅的是以robot作为父坐标系，world为子坐标系
//             listener.waitForTransform("world","robot",now,ros::Duration(1.0));
//             listener.lookupTransform("/world", "/robot",now, transform);
//             saved_rx = transform.getOrigin().x();
//             saved_ry = transform.getOrigin().y();
//             saved_rz = transform.getOrigin().z();
//             ROS_INFO("Transform rotation(RPY) received is %f %f %f",saved_rx,saved_ry,saved_rz);
 
//         }
//         catch (tf::TransformException ex){
//             ROS_ERROR("%s",ex.what());
//             ros::Duration(1.0).sleep();
//         }
//         ROS_INFO("Saved transform is %f %f %f",saved_rx,saved_ry,saved_rz);
//         rate.sleep();
//     }
//   return 0;
// }

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>

class Callback
{
public:
  tf::TransformListener listener_;
  tf::StampedTransform transform_;
  ros::Subscriber sub_;
  double last_x_;
  double last_y_;
  double last_angle_;

  Callback(ros::NodeHandle n)
  {
    last_x_ = 0;
    last_y_ = 0;
    last_angle_ = 0;
    sub_ = n.subscribe<sensor_msgs::LaserScan>("/scan", 1, &Callback::CB, this);
  }
  void CB(const sensor_msgs::LaserScanConstPtr &scan);
};

void Callback::CB(const sensor_msgs::LaserScanConstPtr &scan)
{

  try
  {
    // base_link to map
    listener_.waitForTransform("/map",
                               "/base_link",
                               ros::Time(0), ros::Duration(0.2));
    listener_.lookupTransform("map",
                              "base_link",
                              ros::Time(0), transform_);
  }
  catch (tf::TransformException &ex)
  {
    ROS_ERROR("%s", ex.what());
    return;
  }
  double dx = transform_.getOrigin().x() - last_x_;
  double dy = transform_.getOrigin().y() - last_y_;
  double distance = sqrt(dx * dx + dy * dy);
  tf::Quaternion quat;
  double qx = transform_.getRotation().x(), qy = transform_.getRotation().y(),
         qz = transform_.getRotation().z(), qw = transform_.getRotation().w();
  geometry_msgs::Quaternion qua;
  qua.x = qx;
  qua.y = qy;
  qua.z = qz;
  qua.w = qw;
  tf::quaternionMsgToTF(qua, quat);
  double roll, pitch, yaw;
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  if (yaw >= 6.27)
    yaw = 0;
  double a = (yaw - last_angle_) * 180 / 3.14159;
  if (a < 0)
    a = -a;
  if (a >= 30 || distance >= 1.1)
  {
    ROS_INFO("%lf, %lf", distance, a);
    last_x_ = transform_.getOrigin().x();
    last_y_ = transform_.getOrigin().y();
    last_angle_ = yaw;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lwd_tf");
  ros::NodeHandle n;
  Callback cb(n);
  ros::spin();
  return 0;
}