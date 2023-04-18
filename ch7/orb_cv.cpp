//
// Created by mao on 23-4-13.
//
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <chrono>

using namespace std;
using namespace cv;

//argv[o]是程序的名称，argv[1]是第一个参数，argv[2]是第二个参数
int main(int argc, char **argv)
{
    if (argc != 3)
    {
        cout << "usage: feature_extraction img1 img2" << endl;
        return 1;
    }
    //读取图像文件名，图片加载模式（彩色图）
    Mat img_1 = imread(argv[1], IMREAD_COLOR);
    Mat img_2 = imread(argv[2], IMREAD_COLOR);
    //assert是一个宏，如果条件为真则继续进行，如果为假则终止程序并报错
    assert(img_1.data != nullptr && img_2.data != nullptr);

    //初始化
    vector<KeyPoint> keypoints_1, keypoints_2;//存储特征点
    Mat descriptors_1, descriptors_2;//存储描述子
    Ptr<FeatureDetector> detector = ORB::create();//ORB特征检测器的指针
    Ptr<DescriptorExtractor> descriptor = ORB::create();//ORB特征描述子提取器的指针
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");//暴力匹配器的指针，汉明距离作为匹配度量

    //第一步：检测Oriented FAST 角点位置
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    detector->detect(img_1, keypoints_1);
    detector->detect(img_2, keypoints_2);

    //第二步：根据角点位置计算BRIEF描述子
    descriptor->compute(img_1, keypoints_1, descriptors_1);
    descriptor->compute(img_2, keypoints_2, descriptors_2);
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "extract ORB cost = " << time_used.count() << " seconds." << endl;

    Mat outimg1;//用于存储绘制特征点后的图像
    drawKeypoints(img_1, keypoints_1, outimg1, Scalar::all(-1), DrawMatchesFlags::DEFAULT);
    imshow("ORB features", outimg1);//显示图像

    //第三步：对两幅图中的BRIEF描述子进行匹配，使用汉明距离
    vector<DMatch> matches;
    t1 = chrono::steady_clock::now();
    matcher->match(descriptors_1, descriptors_2, matches);
    t2 = chrono::steady_clock::now();
    time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "match ORB cost = " << time_used.count() << " second. " << endl;

    //第四步匹配点筛选
    //计算最小距离和最大距离
    auto min_max = minmax_element(matches.begin(), matches.end(), //lambda表达式
                                  [](const DMatch &m1, const DMatch &m2) {return m1.distance < m2.distance;});
    double min_dist = min_max.first->distance;//最小元素
    double max_dist = min_max.second->distance;//最大元素

    printf("-- Max dist : %f \n", max_dist);
    printf("-- Min dist : %f \n", min_dist);
    vector<DMatch> good_matches;
    for (int i = 0; i < descriptors_1.rows; i++)
    {
        if (matches[i].distance <= max(2 * min_dist, 30.0))
        {
            good_matches.push_back(matches[i]);
        }
    }

    Mat img_match;
    Mat img_goodmatch;
    drawMatches(img_1, keypoints_1, img_2, keypoints_2,matches, img_match);
    drawMatches(img_1, keypoints_1, img_2, keypoints_2,good_matches, img_goodmatch);
    imshow("all matches", img_match);
    imshow("good matches", img_goodmatch);
    waitKey(0);

    return 0;
}