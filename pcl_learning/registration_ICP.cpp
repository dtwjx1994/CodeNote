//
// Created by lt on 18-6-6.
//

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

/*int
main (int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);

    // Fill in the CloudIn data
    cloud_in->width    = 5;
    cloud_in->height   = 1;
    cloud_in->is_dense = false;
    cloud_in->points.resize (cloud_in->width * cloud_in->height);
    for (size_t i = 0; i < cloud_in->points.size (); ++i)
    {
        cloud_in->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
        cloud_in->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
        cloud_in->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
    }
    std::cout << "Saved " << cloud_in->points.size () << " data points to input:"
              << std::endl;
    for (size_t i = 0; i < cloud_in->points.size (); ++i) std::cout << "    " <<
                                                                    cloud_in->points[i].x << " " << cloud_in->points[i].y << " " <<
                                                                    cloud_in->points[i].z << std::endl;
    *cloud_out = *cloud_in;
    std::cout << "size:" << cloud_out->points.size() << std::endl;
    for (size_t i = 0; i < cloud_in->points.size (); ++i)
        cloud_out->points[i].x = cloud_in->points[i].x + 0.7f;
    std::cout << "Transformed " << cloud_in->points.size () << " data points:"
              << std::endl;
    for (size_t i = 0; i < cloud_out->points.size (); ++i)
        std::cout << "    " << cloud_out->points[i].x << " " <<
                  cloud_out->points[i].y << " " << cloud_out->points[i].z << std::endl;
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputCloud(cloud_in);
    icp.setInputTarget(cloud_out);
    pcl::PointCloud<pcl::PointXYZ> Final;
    icp.align(Final);
    std::cout << "has converged:" << icp.hasConverged() << " score: " <<
              icp.getFitnessScore() << std::endl;
    std::cout << icp.getFinalTransformation() << std::endl;

    return (0);
}*/


#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
using namespace std;
int  main(int argc,char** argv){
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

