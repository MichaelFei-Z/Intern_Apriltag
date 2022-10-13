#include <ros/ros.h>
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <cv_bridge/rgb_colors.h>
#include <vector>
#include <iostream>

class PollingDisplay
{
public:
    PollingDisplay():it(n), index(0)
    {
        msgs_vec = {"/d415/color/tag_detections_image", "/d435/color/tag_detections_image"};
        // msgs_vec = {"/d415/color/image_raw", "/d435/color/image_raw"};

        // 相机显示状态索引数组
        index_vec = std::vector<bool>(msgs_number+1, false);

        cv::namedWindow("view");
        cv::startWindowThread();

        // 图像话题订阅
        image_transport::Subscriber ImageSub_0 = it.subscribe(msgs_vec[0], 20, &PollingDisplay::imageCallback1, this);
        image_transport::Subscriber ImageSub_1 = it.subscribe(msgs_vec[1], 20, &PollingDisplay::imageCallback2, this);

        // 按键处理计时器
        keys_timer = n.createTimer(ros::Duration(0.01), &PollingDisplay::keysTimerCallback, this); 
       // 图像显示计时器 
        show_timer = n.createTimer(ros::Duration(0.5), &PollingDisplay::showTimerCallback, this);
        while (ros::ok())
        {
            ros::spinOnce();
            if(!img2show.empty())
            {
                cv::imshow("view", img2show);
            }
        }
    }

    // 按键处理回调函数
    // 根据键盘输入，更改index_vec（相机显示状态索引数组）
    // 输入指定相机序号（1-n）： 对应相机显示状态置为true，其他置为false
    // 输入 0 ： 表示轮询
    // 输入其他：输出提示非法输入
    void keysTimerCallback(const ros::TimerEvent&)
    {
        int k_value = waitKey(10);
        
        if (k_value != -1)
        {
            if (k_value >= '0' && k_value  <= '0' + msgs_number)
            {
                for (int i=0; i<=msgs_number; ++i)
                {
                    if (k_value == '0' + i)
                    {
                        index_vec[i] = true;
                        ROS_INFO("Press %d", i);
                    }
                    else
                        index_vec[i] = false;
                }
            }
            else
            {
                ROS_INFO("Please press keys from 0 - %d. ", msgs_number);
            }
        }
    }

    // 图像显示回调函数（通过更改index（相机序号）来控制显示）
    // 依据index_vec（相机显示状态索引数组）更改index值
    // index_vec[0] = true : 轮询，自加取余实现
    // index_vec[i] = true ： 显示序号为 （i-1） 的相机（从0开始编号）
    void showTimerCallback(const ros::TimerEvent&)
    {
        if (index_vec[0] == true)
        {
            index = (index + 1) % msgs_number;
            ROS_INFO(" Polling Show ... ");
        }
        else
        {
            for (int i=1; i<=msgs_number; ++i)
            {
                if (index_vec[i] == true)
                {
                    index = i-1;
                    ROS_INFO(" Freeze Show camera %d.", i);
                }
            }
        }
    }

    void imageCallback1(const sensor_msgs::ImageConstPtr& img_msg)
    {
        if (index == 0)
            cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8)->image.copyTo(img2show);
    }

    void imageCallback2(const sensor_msgs::ImageConstPtr& img_msg)
    {
        if (index == 1)
            cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8)->image.copyTo(img2show);
    }

    ~PollingDisplay()
    {
        cv::destroyWindow("view");
    }

private:
    static int msgs_number;

    int index;
    std::vector<bool> index_vec;
    ros::NodeHandle n; 
    image_transport::ImageTransport it;
    std::vector<std::string> msgs_vec;
    cv::Mat img2show;
    ros::Timer keys_timer;
    ros::Timer show_timer;
};

int PollingDisplay::msgs_number = 2;
 

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_listener");

    PollingDisplay polling_display;

    return 0;
}