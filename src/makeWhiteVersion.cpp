#include <ros/ros.h>
#include <Eigen/Dense>
#include <quad_tree_decimation/quad_tree/quad_tree.h>
#include <iostream>
#include <fstream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
using namespace std;


using namespace QTD;



int main(int argc, char **argv) {

    // ros::init(argc, argv, "quadTreeExample");

    // ros::Rate loop_rate(10);

    std::string basepath = "/home/unnar/catkin_ws/src/quadtree_example/data/";
    std::string subpath = "comparison/";
    std::string name = "errored_cloud_EfficientPPR_020_201";

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>());

    pcl::PCDReader reader;
    reader.read (basepath + subpath + name +".pcd", *cloud);

    for(auto &p : cloud->points){
        if(255 - p.r < 2 && 255 - p.g < 2 && 255 - p.b < 2){
            p.r = 0;
            p.g = 0;
            p.b = 0;
        }
    }

    pcl::PCDWriter writer;
    writer.write(basepath + subpath + name +"_white.pcd", *cloud);



    return 0;
}
