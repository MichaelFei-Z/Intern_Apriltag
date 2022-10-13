#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include <algorithm>

using namespace cv;
using namespace std;


class PollingDisplay
{
public:
    PollingDisplay():index(-1), poll_id(0)
    {
        index_vec = std::vector<bool>(msgs_number+1, false);
        cv::namedWindow("view");
        cv::startWindowThread();
        while(1)
        {
            keysTimerCallback();
            showTimerCallback();
            imageCallback1();
            imageCallback2();
            imageCallback3();
            imageCallback4();

            if(!img2show.empty())
            {
                cv::imshow("view", img2show);
            }
        }
    }

    void keysTimerCallback()
    {
        int k_value = waitKey(500);
        
        if (k_value != -1)
        {
            // 按键 r ： 清除所有显示
            if (k_value == 'r')
            {
                index_vec = std::vector<bool>(msgs_number, false);
                show_index_vec = {};
            }


            if (k_value >= '0' && k_value  <= '0' + msgs_number)
            {
                for (int i=0; i<=msgs_number; ++i)
                {
                    if (k_value == '0' + i)
                    {
                        // 第一次按下，代表确认当前视角显示
                        if (index_vec[i] == false)
                        {
                            index_vec[i] = true;
                            cout << "Press " << i << endl;
                        }
                        // 第二次按下，代表取消当前视角显示
                        else
                        {
                            index_vec[i] = false;
                            cout << "Remove " << i << endl;
                        }
                    }
                }
            }
            else
            {
                cout << "Please press keys from 0 - " << msgs_number << ". " << endl;
            }
        }
    }

    void showTimerCallback()
    {
        // 相机状态为 true， 即代表要显示，加入显示索引数组中
        // 相机状态为 false，即代表不显示，从显示索引数组中剔除
        for (int i=0; i<=msgs_number; ++i)
        {
            if (index_vec[i] == true)
                show_index_vec.push_back(i-1);
            else
                std::remove(show_index_vec.begin(), show_index_vec.end(), i-1);
        }

        // 为防止重复加入，进行排序去重操作
        std::sort(show_index_vec.begin(), show_index_vec.end());
        show_index_vec.erase(unique(show_index_vec.begin(), show_index_vec.end()), show_index_vec.end());


        if (!show_index_vec.empty())
        {
            // 打印提示 显示列表
            cout << "Show Camera list: ";
            for (auto id : show_index_vec)
                cout << id << ", ";
            cout << endl;

            poll_id = (poll_id+1) % show_index_vec.size();
            index = show_index_vec[poll_id];
            cout << " Polling Show camera " << index << endl;
        }
        else
        {
            cout << "Show Camera list: empty ." << endl;
        }
    }

    void imageCallback1()
    {
        if (index == 0)
            img2show = imread("../1.png");
    }

    void imageCallback2()
    {
        if (index == 1)
            img2show = imread("../2.png");
    }

    void imageCallback3()
    {
        if (index == 2)
            img2show = imread("../3.png");
    }

    void imageCallback4()
    {
        if (index == 3)
            img2show = imread("../4.png");
    }

    ~PollingDisplay()
    {
        cv::destroyWindow("view");
    }

private:
    static int msgs_number;
    int index;
    int poll_id;
    std::vector<bool> index_vec;
    std::vector<int> show_index_vec;
    cv::Mat img2show;
};
int PollingDisplay::msgs_number = 4;


int main(int argc, char **argv)
{
    PollingDisplay polling_display;

    return 0;
}
