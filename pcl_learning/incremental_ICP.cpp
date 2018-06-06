//
// Created by lt on 18-6-6.
//

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/point_representation.h>
#include <boost/make_shared.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/filter.h>
#include <pcl/features/normal_3d.h>
using namespace std;
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;
pcl::visualization::PCLVisualizer *p;
int vp_1,vp_2;

struct PCD{
    PointCloud::Ptr cloud;
    string f_name;
    PCD():cloud(new PointCloud){};
};

void pairAlign(const PointCloud::Ptr cloud_src,const PointCloud::Ptr cloud_tgt,PointCloud::Ptr output,Eigen::Matrix4f &final_transform,bool downsample=false){
    PointCloud::Ptr src (new PointCloud);
    PointCloud::Ptr tgt(new PointCloud);
    pcl::VoxelGrid<PointT> grid;

    if(downsample){
        grid.setLeafSize(0.05,0.05,0.05);
        grid.setInputCloud(cloud_src);
        grid.filter(*src);

        grid.setInputCloud(cloud_tgt);
        grid.filter(*tgt);
    }else{
        src=cloud_src;
        tgt=cloud_tgt;
    }

    // 计算 表面分布和 曲率
    PointCloudWithNormals::Ptr points_with_normals_src (new PointCloudWithNormals);
    PointCloudWithNormals::Ptr Points_with_normals_src (new PointCloudWithNormals);
    pcl::NormalEstimation<PointT,PointNormalT> norm_est;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<PointT>);
    norm_est.setSearchMethod(tree);
    norm_est.setKSearch(30)

}

void loadData(vector<string> path, std::vector<PCD,Eigen::aligned_allocator<PCD>> &models){
    std::string extension(".pcd");
    for (int i = 0; i <path.size() ; ++i) {
        std::transform(path[i].begin(),path[i].end(),path[i].begin(),(int(*)(int))tolower);
        PCD m;
        m.f_name=path[i];
        pcl::io::loadPCDFile(m.f_name,*m.cloud);
        vector<int> indices;
        pcl::removeNaNFromPointCloud(*m.cloud,*m.cloud,indices);
        models.push_back(m);
    }



}

int  main(int argc,char** argv){
    vector<PCD,Eigen::aligned_allocator<PCD>> data;
    loadData(data);
    p=new pcl::visualization::PCLVisualizer(argc,argv," vis ");
    p->createViewPort(0.0,0,0.5,1.0,vp_1);
    p->createViewPort(0.5,0,1.0,1.0,vp_2);

    PointCloud::Ptr result(new PointCloud),source,target;
    Eigen::Matrix4f GlobalTransform=Eigen::Matrix4f::Identity(),pairTranform;

    for (int i = 1; i < data.size(); ++i) {
        source=data[i-1].cloud;
        target=data[i].cloud;

        PointCloud::Ptr temp(new PointCloud);
        pairAlign(source,target,temp,pairTransform,true);


    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud2(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCDReader reader;
    reader.read("sample.pcd",*pointCloud1);
    *pointCloud2=*pointCloud1;
    for (int i = 0; i < pointCloud2->points.size(); ++i) {
        pointCloud2->points[i].x+=0.3;
        pointCloud2->points[i].y-=0.2;
    }
    pcl::IterativeClosestPoint<pcl::PointXYZ,pcl::PointXYZ> icp;
    icp.setInputSource(pointCloud1);
    icp.setInputTarget(pointCloud2);
    pcl::PointCloud<pcl::PointXYZ> final;
    icp.align(final);
    std::cout << "has converged:" << icp.hasConverged() << " score: " <<
              icp.getFitnessScore() << std::endl;
    std::cout<<icp.getFitnessScore()<<" "<<endl;
    std::cout<<icp.getFinalTransformation()<<" "<<endl;
    return 0;
}