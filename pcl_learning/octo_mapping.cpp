//
// Created by lt on 18-6-4.
//

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <boost/format.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace std ;

// 全局变量：相机矩阵
// 更好的写法是存到参数文件中，但为方便起见我就直接这样做了
float camera_scale  = 1000;
float camera_cx     = 325.5;
float camera_cy     = 253.5;
float camera_fx     = 518.0;
float camera_fy     = 519.0;
int main(int argc,char ** argv){
    ifstream fin("./keyframe.txt");
    vector<int>keyframe;
    vector<Eigen::Isometry3d> poses;

    while (fin.peek()!=EOF){
        int index_keyframe;
        fin>>index_keyframe;
        if(fin.fail())
            break;
        keyframe.push_back(index_keyframe);
    }
    fin.close();
    // 读g2o文件
    fin.open("");
    while(fin.peek()!=EOF){
        int index_keyframe;
        float data[7];
        fin>>index_keyframe;
        for (int i = 0; i < 7; ++i) {
            fin>>data[i];
            cout<<data[i]<<" ";
        }
        Eigen::Quaterniond q(data[6],data[3],data[4],data[5]);// qw qx qy qz
        Eigen::Isometry3d t(q);
        t(0,3)=data[0];
        t(1,3)=data[1];
        t(2,3)=data[2];
        poses.push_back(t);
    }
    fin.close();
    octomap::ColorOcTree tree(0.01);
    for (int i = 0; i < keyframe.size(); ++i) {
        pcl::PointCloud<pcl::PointXYZRGBA> cloud;
        int k=keyframe[i];
        Eigen::Isometry3d& pose=poses[i];
        boost::format fmt(" ");
        cv::Mat depth=cv::imread((fmt%k).str().c_str(),-1);

        for (int j = 0; j < depth.rows; ++j) {
            for (int l = 0; l < depth.cols; ++l) {
                ushort d=depth.ptr<ushort >(j)[l];
                if (d==0)
                    continue;
                float z=float(d)/camera_scale;
                float x=(l-camera_cx) * z/camera_fx;
                float y=(j-camera_cy) * z/camera_fy;

                pcl::PointXYZRGBA p;
            }
        }
    }
    string input_file="sample.pcd";
    string output_file="table.ot";

//    pcl::io::loadPCDFile<pcl::PointXYZRGBA>(input_file,cloud);
//    for (int i = 0; i < cloud.points.size(); ++i) {
//        tree.updateNode(octomap::point3d(cloud.points[i].x,cloud.points[i].y,cloud.points[i].z), true); //true表示这个点是占据点，false 空闲点
//    }
//    for (int i = 0; i < cloud.points.size(); ++i) {
//        tree.integrateNodeColor(cloud.points[i].x,cloud.points[i].y,cloud.points[i].z ,cloud.points[i].r,cloud.points[i].g,cloud.points[i].b); //true表示这个点是占据点，false 空闲点
//    }
    tree.updateInnerOccupancy();
    tree.write(output_file);
}