//
// Created by lt on 18-6-2.
//

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
int main() {

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    cloud->width=5;
    cloud->height=1;
    cloud->is_dense= false;
    cloud->points.resize(cloud->width*cloud->height);

    for(int i=0;i<cloud->points.size();++i){
        cloud->points[i].x=1024*rand()/(RAND_MAX+1.0);
        cloud->points[i].y=1024*rand()/(RAND_MAX+1.0);
        cloud->points[i].z=1024*rand()/(RAND_MAX+1.0);
    }
    std::cerr<<"clould before filter "<<std::endl;
    for(size_t i=0;i<cloud->points.size();++i){
        std::cerr<<cloud->points[i].x<<" "<<cloud->points[i].y<<" "<<cloud->points[i].z<<std::endl;
    }
    std::cout<<"after"<<std::endl;
    //filter
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0,1);
//    pass.setNegative();
    pass.filter(*cloud_filtered);
    for (int j = 0; j <cloud_filtered->points.size(); ++j) {
        std::cerr<<cloud_filtered->points[j].x<<" "<<cloud_filtered->points[j].y<<" "<<cloud_filtered->points[j].z<<std::endl;
    }

//
//    pcl::io::savePCDFileASCII("test_pcd.pcd",cloud);
//    std::cerr<<"Saved: "<<cloud.points.size()<<std::endl;

    return 0;
}