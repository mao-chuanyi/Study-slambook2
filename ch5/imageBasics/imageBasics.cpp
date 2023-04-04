//
// Created by mao on 23-4-2.
//
#include <iostream>
#include <chrono>

using namespace std;

#include <opencv4/opencv2/core/core.hpp>
#include <opencv4/opencv2/highgui/highgui.hpp>


int  main(int argc, char **argv){
    cv::Mat image;
    image = cv::imread(argv[1]);

    if(image.data == nullptr){
        cerr << "file" << argv[1] << "is not exist" << endl;
        return 0;
    }

    cout << "wight:" << image.cols << ", height:" << image.rows
         << ", count of channels:" << image.channels() << endl;
    cv::imshow("image", image);
    cv::waitKey(0);

    if (image.type() != CV_8UC1 && image.type() != CV_8UC3){
        cout << "Please enter a color or grayscale map" << endl;
        return 0;
    }

    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    for(size_t y = 0; y < image.rows; y++){
        unsigned char *row_ptr = image.ptr<unsigned char>(y);
        for(size_t x = 0; x < image.cols; x++) {
            unsigned char *data_ptr = &row_ptr[x * image.channels()];
            for (int c = 0; c != image.channels(); c++){
                unsigned char data = data_ptr[c];
            }
        }
    }
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "time used = " << time_used.count() << "s" << endl;

    // 关于 cv::Mat 的拷贝
    // 直接赋值并不会拷贝数据
    cv::Mat image_another = image;
    // 修改 image_another 会导致 image 发生变化
    image_another(cv::Rect(0, 0, 100, 100)).setTo(0); // 将左上角100*100的块置零
    cv::imshow("image", image);
    cv::waitKey(0);

    cv::Mat image_clone = image.clone();
    image_clone(cv::Rect(0,0,100,100)).setTo(255);
    cv::imshow("image", image);
    cv::imshow("image_clone", image_clone);
    cv::waitKey(0);
    cv::destroyAllWindows();


    return 0;
}