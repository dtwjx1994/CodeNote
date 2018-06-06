//
// Created by sc on 18-6-6.
//

//  https://github.com/gaoxiang12/slambook/blob/master/ch5/joinMap/joinMap.cpp
#include <iostream>
#include <fstream>
using namespace std;

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <Eigen/Geometry>

#include <boost/format.hpp>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

int main(int argc,char **argv){
    vector<cv::Mat>colorImgs,depthImgs;
    vector<Eigen::Isometry3d,Eigen::aligned_allocator<Eigen::Isometry3d>>poses;
    ifstream fin("./pose.txt");

    for (int i = 0; i < 5; ++i) {
        boost::format fmt("./%s/%d.%s");
        colorImgs.push_back(cv::imread((fmt%"color"%(i+1)%"png").str()));
        depthImgs.push_back(cv::imread((fmt%"depth"%(i+1)%"pgm").str(),-1)); //-1表示原始图像

        double data[7]={0};
        for(auto &d:data){
            fin>>d;
        }
        Eigen::Quaterniond q(data[6],data[3],data[4],data[5]);
        Eigen::Isometry3d T(q);
        T.pretranslate(Eigen::Vector3d(data[0],data[1],data[2]));

        poses.push_back(T);
    }

    double cx=325.5;
    double cy=253.5;
    double fx=518.0;
    double fy=519.0;

    double depthScale=1000.0;

    cout<<"拼接"<<endl;
    typedef pcl::PointXYZRGB PointT;
    typedef pcl::PointCloud<PointT> PointCloud;


}
