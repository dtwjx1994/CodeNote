//
// Created by lt on 18-6-4.
//

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>
using namespace std ;
int main(int argc,char ** argv){
    string input_file="sample.pcd";
    string output_file="table.ot";

    pcl::PointCloud<pcl::PointXYZRGBA> cloud;
    pcl::io::loadPCDFile<pcl::PointXYZRGBA>(input_file,cloud);
    octomap::ColorOcTree tree(0.01);
    for (int i = 0; i < cloud.points.size(); ++i) {
        tree.updateNode(octomap::point3d(cloud.points[i].x,cloud.points[i].y,cloud.points[i].z), true); //true表示这个点是占据点，false 空闲点
    }
    for (int i = 0; i < cloud.points.size(); ++i) {
        tree.integrateNodeColor(cloud.points[i].x,cloud.points[i].y,cloud.points[i].z ,cloud.points[i].r,cloud.points[i].g,cloud.points[i].b); //true表示这个点是占据点，false 空闲点
    }
    tree.updateInnerOccupancy();
    tree.write(output_file);
}