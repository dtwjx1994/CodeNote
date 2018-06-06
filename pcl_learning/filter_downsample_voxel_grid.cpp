//
// Created by lt on 18-6-2.
//

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
using namespace std;
int main() {
    pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2);
    pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2);
    pcl::PCDReader reader;
    string path="table_scene_lms400.pcd";
    reader.readHeader("table_scene_lms400.pcd",*cloud);
    reader.read("table_scene_lms400.pcd",*cloud);
    cerr<<cloud->width<<" "<<cloud->height<<pcl::getFieldsList(*cloud)<<endl;

    std::cout<<"after"<<std::endl;
    //filter
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(0.05f,0.05f,0.05f);
    sor.filter(*cloud_filtered);
    pcl::PCDWriter writer;
    writer.write("table_sense_downsampled.pcd",*cloud_filtered,Eigen::Vector4f::Zero(),Eigen::Quaternionf::Identity(),
                 false);

//
//    pcl::io::savePCDFileASCII("test_pcd.pcd",cloud);
//    std::cerr<<"Saved: "<<cloud.points.size()<<std::endl;

    return 0;
}
