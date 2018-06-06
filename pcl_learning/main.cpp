#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
int main() {
    std::cout << "Hello, World!" << std::endl;
    pcl::PointCloud <pcl::PointXYZ> cloud;
    cloud.width=5;
    cloud.height=1;
    cloud.is_dense= false;
    cloud.points.resize(cloud.width*cloud.height);

    for(int i=0;i<cloud.points.size();++i){
        cloud.points[i].x=1024*rand()/(RAND_MAX+1.0);
        cloud.points[i].y=1024*rand()/(RAND_MAX+1.0);
        cloud.points[i].z=1024*rand()/(RAND_MAX+1.0);
    }
    pcl::io::savePCDFileASCII("test_pcd.pcd",cloud);
    std::cerr<<"Saved: "<<cloud.points.size()<<std::endl;

    return 0;
}