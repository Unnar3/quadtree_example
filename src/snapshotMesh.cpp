#include <ros/ros.h>
#include <quad_tree_decimation/plane_extraction/plane_extraction.h>
#include <quad_tree_decimation/quad_tree/quad_tree_pcl.h>
#include <quad_tree_decimation/quad_tree/quad_tree.h>
#include <quad_tree_decimation/point_types/surfel_type.h>
#include <quad_tree_decimation/utils/utils.h>
#include <exx_compression/compression.h>
#include <pcl/kdtree/kdtree_flann.h>
// #include <exx_compression/planes.h>
#include <metaroom_xml_parser/simple_xml_parser.h>
// #include <PointTypes/surfel_type.h>
// #include <pcl/visualization/pcl_visualizer.h>
// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/sample_consensus/sac_model_plane.h>

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <algorithm>
#include <opencv2/opencv.hpp>


// DEFINITIONS
#define NODE_NAME               "snapshot_mesh"

using PointT = pcl::PointXYZRGB;
using PointCloudT = pcl::PointCloud<PointT>;
using NormalT = pcl::Normal;
using NormalCloudT = pcl::PointCloud<NormalT>;
using PointS = SurfelType;
using PointCloudS = pcl::PointCloud<PointS>;

using namespace QTD::Parameters;
// using namespace EXX;

class SnapShot
{
public:
    ros::NodeHandle nh;
private:
    // ros::Publisher point_cloud_publisher;
    EXX::compression cmprs;
    QTD::planeExtraction planeEx;
    primitive_params params;

public:

    SnapShot()
    {
        nh = ros::NodeHandle("~");
        cmprs.setVoxelLeafSize(loadParam<double>("VoxelLeafSize", nh));
        cmprs.setRWHullMaxDist(loadParam<double>("RWHullMaxDist", nh));
        cmprs.setHULLAlpha(loadParam<double>("hullAlpha", nh));

        QTD::QuadTreePCL<pcl::PointXYZ> qtpcl(1,10,0,0);

        params.number_disjoint_subsets = loadParam<int>("disjoinedSet", nh);
        params.octree_res              = loadParam<double>("octree_res", nh);
        params.normal_neigbourhood     = loadParam<double>("normal_neigbourhood", nh);
        params.inlier_threshold        = loadParam<double>("inlier_threshold", nh);
        params.angle_threshold         = loadParam<double>("angle_threshold", nh);
        params.add_threshold           = loadParam<double>("add_threshold", nh);
        params.connectedness_res       = loadParam<double>("connectedness_res", nh);
        params.distance_threshold      = loadParam<double>("distance_threshold", nh);
        params.inlier_min              = loadParam<int>("inlier_min", nh);
        params.min_shape               = loadParam<int>("min_shape", nh);

        planeEx.setPrimitiveParameters(params);
        // std::cout << "leaf size: " << cmprs.getVoxelLeafSize() << std::endl;
    }

    void createMesh(void){

        // PointCloudS::Ptr cloud (new PointCloudS());

        // std::string dataset_path = loadParam<std::string>("dataset_path", nh);
        // std::string dataset_cloud = loadParam<std::string>("dataset_cloud", nh);
        // std::string dataset_save = loadParam<std::string>("dataset_save", nh);
        // std::string save_prefix = dataset_cloud.substr(0,dataset_cloud.find_last_of('/'));
        // std::cout << save_prefix << std::endl;
        // std::replace(save_prefix.begin(), save_prefix.end(), '/', '_');


        std::string test_cloud_path = loadParam<std::string>("dataset_path", nh);
        std::cout << "test_cloud_path: " << test_cloud_path << std::endl;
        std::string test_cloud_name = loadParam<std::string>("dataset_cloud", nh);
        std::cout << "test_cloud_name: " << test_cloud_name << std::endl;
        std::string test_cloud = test_cloud_path + test_cloud_name;
        std::cout << "test_cloud: " << test_cloud << std::endl;
        std::string save_path = test_cloud.substr(0,test_cloud.find_last_of('/')) + '/';
        std::cout << "save_path: " << save_path << std::endl;
        std::string mesh_name = test_cloud.substr(test_cloud.find_last_of('/')+1);
        mesh_name = mesh_name.substr(0, mesh_name.find_last_of('.')) + ".obj";
        std::cout << "mesh_name: " << mesh_name << std::endl;



        // cv::Mat image = cv::Mat::zeros(10,10,CV_16UC3);
        // image.at<cv::Vec3f>(0,0)[0] += 200.0;
        // // double i = image.at<cv::Vec3f>(0,0)[0] + 200.0;
        // // std::cout << "i = " << i << std::endl;
        // // image.at<cv::Vec3f>(0,0)[0] = i;
        // // double k = image.at<cv::Vec3f>(0,0)[0];
        // std::cout << "image pixel 1 R: "  << std::to_string((int)image.at<cv::Vec3f>(0,0)[0])  << std::endl;
        // image.at<cv::Vec3f>(0,0)[0] += 150;
        // std::cout << "image pixel 1 R: "  << std::to_string((int)image.at<cv::Vec3f>(0,0)[0])  << std::endl;
        //
        //
        // exit(0);


        std::cout << save_path << std::endl;

        PointCloudT::Ptr segment (new PointCloudT());
        // PointCloudT::Ptr segment_v (new PointCloudT());
        NormalCloudT::Ptr normals (new NormalCloudT());
        pcl::PCDReader reader;
        reader.read (test_cloud, *segment);
        // cmprs.voxelGridFilter(segment, segment_v);


        // std::cout << "surfel_cloud: " << surfel_cloud->points.size() << std::endl;

        // create regular pointcloud and normal cloud.
        // normals->reserve(surfel_cloud->size());
        // segment->reserve(surfel_cloud->size());


        // Can't remember if this is really needed :O
        // for(auto p : surfel_cloud->points){
        //     if(p.confidence >= 0.0){
        //         PointT pt;
        //         NormalT pn;
        //         pt.x = p.x;
        //         pt.y = p.y;
        //         pt.z = p.z;
        //         pt.r = (p.rgba >> 16) & 0x0000ff  ;
        //         pt.g = (p.rgba >> 8)  & 0x0000ff;
        //         pt.b = (p.rgba)       & 0x0000ff;
        //         pn.normal_x = p.normal_x;
        //         pn.normal_y = p.normal_y;
        //         pn.normal_z = p.normal_z;
        //         normals->push_back(pn);
        //         segment->push_back(pt);
        //     }
        // }

        pcl::PCDWriter writer;
        // writer.write(save_path + "threshholded.pcd", *segment);
        // exit(0);


        ////////////////////////////////////////////////////////////////////////
        // EFFICIENT RANSAC with PPR
        ////////////////////////////////////////////////////////////////////////

        PointCloudT::Ptr nonPlanar (new PointCloudT());

        std::vector<PointCloudT::Ptr> plane_vec;
        std::vector<pcl::ModelCoefficients::Ptr> normal_vec;

        std::cout << "Efficient PPR..................." << std::endl;
        // planeDetection::planeSegmentationEfficientPPR(segment, params, plane_vec, normal_vec, nonPlanar);
        std::cout << "segment size: " << segment->points.size() << std::endl;

        while(true){
            std::vector<PointCloudT::Ptr> plane_vec_tmp;
            std::vector<pcl::ModelCoefficients::Ptr> normal_vec_tmp;

            planeEx.planeSegmentationEfficientPPR(segment, normals, plane_vec_tmp, normal_vec_tmp, nonPlanar);

            std::cout << "size: " << plane_vec_tmp.size() << std::endl;

            plane_vec.insert(plane_vec.end(), plane_vec_tmp.begin(), plane_vec_tmp.end());
            normal_vec.insert(normal_vec.end(), normal_vec_tmp.begin(), normal_vec_tmp.end());
            *segment = *nonPlanar;
            if(plane_vec_tmp.size() < 2) break;

            params.angle_threshold = params.angle_threshold + params.angle_threshold/2.0;
            params.inlier_min = params.inlier_min * 2.0;
            params.min_shape = params.inlier_min;
            planeEx.setPrimitiveParameters(params);
            break;
        }


        PointCloudT::Ptr outCloudEfficientPPR( new PointCloudT() );
        planeEx.combinePlanes(plane_vec, outCloudEfficientPPR, true);

        PointCloudT::Ptr outCloudEfficientPPRnc( new PointCloudT() );
        planeEx.combinePlanes(plane_vec, outCloudEfficientPPRnc, false);

        ofstream myfile;
        myfile.open (save_path + "segmented.txt");
        int n = 0;
        for(auto plane: plane_vec){
            myfile << "Plane " + std::to_string(n) + ": " + std::to_string(plane->size()) << std::endl;
            n++;
        }
        myfile.close();




        // PROJECT TO PLANE
        std::cout << "hmmm" << std::endl;

        planeEx.projectToPlane<PointT>( plane_vec, normal_vec );
        std::cout << "ha" << std::endl;
        planeEx.mergePlanes<PointT>( plane_vec, normal_vec );
        std::cout << "ho" << std::endl;
        planeEx.projectToPlane<PointT>( plane_vec, normal_vec );
        std::cout << "hi" << std::endl;

        writer.write(save_path + "segmented.pcd", *outCloudEfficientPPR);
        writer.write(save_path + "segmented_nc.pcd", *outCloudEfficientPPRnc);

        // EXX::compression cmprs;
        // cmprs.setRWHullMaxDist(0.02);
        // cmprs.setHULLAlpha(0.07);

        std::vector<PointCloudT::Ptr> hulls;
        cmprs.planeToConcaveHull(&plane_vec, &hulls);

        std::cout << "plane size: " << plane_vec.size() << std::endl;
        std::cout << "hulls size: " << hulls.size() << std::endl;


        // std::vector<EXX::densityDescriptor> dDesc;
        // EXX::densityDescriptor dens;
        // dens.rw_max_dist = loadParam<double>("RWHullMaxDist", nh);
        // dDesc.push_back(dens);
        // std::vector<PointCloudT::Ptr> simplified_hulls;
        // cmprs.reumannWitkamLineSimplification( &hulls, &simplified_hulls, dDesc);



        // for(int i = 0; i < hulls.size(); ++i){
        //     for(int j = 0; j < hulls[i]->points.size(); ++j){
        //         hulls[i]->at(j).r = 255;
        //         hulls[i]->at(j).g = 255;
        //         hulls[i]->at(j).b = (int)((float)j/(float)hulls[i]->points.size()*(float)255);
        //     }
        //     std::cout << "hull size: " << hulls[i]->size() << std::endl;
        //     writer.write(save_path + "/hulls/hull_" + std::to_string(i) +  ".pcd", *hulls[i]);
        // }

        // for(auto &c : hulls){
        //     int k = 0;
        //     for(auto &p : c->points){
        //         p.r = 255;
        //         p.g = 255;
        //         p.b = k/c->points.size()*255;
        //         k++;
        //     }
        //     writer.write(save_path + "/hulls/hull_" + std::to_string(j) +  ".pcd", *c);
        //     j++;
        // }
        // int j = 0;
        // for(auto c : plane_vec){
        //     std::cout << "plane size: " << c->size() << std::endl;
        //     writer.write(save_path + "/hulls/plane_" + std::to_string(j) +  ".pcd", *c);
        //     j++;
        // }




        QTD::objStruct<PointT> object(1);
        for ( size_t i = 0; i < plane_vec.size(); ++i ){
        // for ( size_t i = 0; i < 2; ++i ){

            QTD::QuadTreePCL<PointT> qtpcl(0.01);
            // qtpcl.setMaxLevel(10);
            qtpcl.setMaxWidth(1);
            qtpcl.setNormal(normal_vec[i]);

            PointCloudT::Ptr out (new PointCloudT());
            std::vector< pcl::Vertices > vertices;
            std::cout << "insert" << std::endl;
            qtpcl.insertBoundary(hulls[i]);
            std::cout << "mesh" << std::endl;
            qtpcl.createMeshNew<PointT>(out, vertices);

            cv::Mat image;
            std::vector<Eigen::Vector2f> vertex_texture;
            std::cout << "texture" << std::endl;
            qtpcl.createTexture<PointT>(plane_vec[i], out, image, vertex_texture);

            object.clouds.push_back(out);
            object.polygons.push_back(vertices);
            object.images.push_back(image);
            object.texture_vertices.push_back(vertex_texture);
            object.coefficients.push_back(normal_vec[i]);

        }
        QTD::saveOBJFile(save_path + "mesh.obj", object, 5);






    }


};

int main(int argc, char **argv) {

    ros::init(argc, argv, NODE_NAME);

    SnapShot dMesh;

    ros::Rate loop_rate(10);
    dMesh.createMesh();
    return 0;
}
