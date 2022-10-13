#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>

using namespace cv;
using namespace std;


class PollingDisplay
{
public:
    PollingDisplay():index(0)
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
            // if (k_value == 'r')
            // {
            //     index_vec[0] = true;
            //     for (int i=1; i<=msgs_number; ++i)
            //         index_vec[i] = false;
            // }

            if (k_value >= '0' && k_value  <= '0' + msgs_number)
            {
                for (int i=0; i<=msgs_number; ++i)
                {
                    if (k_value == '0' + i)
                    {
                        index_vec[i] = true;
                        cout <<"Press " << i << endl;
                    }
                    else
                        index_vec[i] = false;
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
        if (index_vec[0] == true)
        {
            index = (index + 1) % msgs_number;
            cout << " Polling Show ... " << endl;
        }
        else
        {
            for (int i=1; i<=msgs_number; ++i)
            {
                if (index_vec[i] == true)
                {
                    index = i-1;
                    cout << " Freeze Show camera " << i << endl;
                }
            }
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
    std::vector<bool> index_vec;
    cv::Mat img2show;
};
int PollingDisplay::msgs_number = 4;


int main(int argc, char **argv)
{
    PollingDisplay polling_display;

    return 0;
}


// int main(int argc, char**argv)
// {
//     Mat img;
//     img = imread("../test.png");

//     cout << (int)'1' <<endl;

//     int k_value = -1;
//     while (1)
//     {
//         k_value = waitKey(1000);
//         imshow("test", img);
//         cout << k_value << endl;
//     }
    
//     return 0;
// }





 

