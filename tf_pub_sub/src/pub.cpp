#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
int main(int argc, char** argv){
    //初始化发布节点，demo_tf_broadcaster是节点的名字
    ros::init(argc, argv, "demo_tf_broadcaster");
    //创建一个tf发布对象
    static tf::TransformBroadcaster br;
    while(ros::ok())
    {
      //创建一个tf对象
      tf::Transform transform;
      //设置tf的位移量
      transform.setOrigin( tf::Vector3(0.3369507, -0.37187648, -1.80606454));
      //初始化一个四元数变量
      tf::Quaternion q;
      //设置俯仰角
      q.setRPY(0.93040044, 0.43019939, -3.08609162);
      //将四元数存入tf对象中，tf只能存储四元数。
      transform.setRotation(q);
      //"robot" 是父坐标，"world"是子坐标
      //转换关系是从robot坐标系转到world坐标系
      br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "robot"));
      ROS_INFO("The transform has broadcasted!");
      ros::Duration(0.05).sleep();
    }
    return 0;
};