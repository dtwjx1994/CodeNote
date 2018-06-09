//
// Created by sc on 18-6-8.
//
#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <boost/format.hpp>
int
main (int argc, char** argv){
    pcl::PCLPointCloud2::Ptr cloud_blob (new pcl::PCLPointCloud2),cloud_filtered_blob (new pcl::PCLPointCloud2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>),
            cloud_p(new pcl::PointCloud<pcl::PointXYZ>),
            cloud_f(new pcl::PointCloud<pcl::PointXYZ>);


    pcl::PCDReader reader;
    reader.read("table_scene_lms400.pcd",*cloud_blob);
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(cloud_blob);
    sor.setLeafSize(0.01f,0.01,0.01);
    sor.filter(*cloud_filtered_blob);

    pcl::fromPCLPointCloud2(*cloud_filtered_blob,*cloud_filtered);

    pcl::PCDWriter writer;
    writer.write("table_down_sample.pcd",*cloud_filtered,false);


    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setDistanceThreshold(0.01);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    int i=0,nr_points=(int) cloud_filtered->points.size();

    while(cloud_filtered->points.size()>0.1*nr_points){
        seg.setInputCloud(cloud_filtered);
        seg.segment(*inliers,*coefficients);
        if(inliers->indices.size()==0){
            std::cerr<<" err"<<std::endl;
            break;
        }
        extract.setInputCloud(cloud_filtered);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*cloud_p);
        boost::format fmt("table_scene_lms400_plane_%1%.pcd");
        fmt%i;
        std::cout<<fmt.str();
        writer.write(fmt.str(),*cloud_p, false);

        extract.setNegative(true);
        extract.filter(*cloud_f);
        cloud_filtered.swap(cloud_f);
        i++;

    }

    return 0;
}