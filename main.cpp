#include <QCoreApplication>
#include<opencv2\opencv.hpp>
#include<pcl/io/io.h>
#include<pcl/io/pcd_io.h>//pcd 读写类相关的头文件。
#include<pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include<Eigen/Dense>
#include <planefitting.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>



int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    DL_Plane dl_plane;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ>("./result.pcd",*cloud);
    dl_plane.showCloud("原点云",cloud);

    /*///////////////////////////////////////PCL提供的平面拟合////////////////////////////////////////*/
//    pcl::VoxelGrid<pcl::PointXYZ> vg;
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
//    vg.setInputCloud (cloud);
//    vg.setLeafSize (0.1f, 0.1f, 0.1f);
//    vg.filter (*cloud_filtered);

//    std::cout << "PointCloud after filtering has: " << cloud_filtered->size ()  << " data points." << std::endl; //*
//    // Create the segmentation object for the planar model and set all the parameters
//    pcl::SACSegmentation<pcl::PointXYZ> seg;
//    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
//    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
//    seg.setOptimizeCoefficients (true);
//    seg.setModelType (pcl::SACMODEL_PLANE);
//    seg.setMethodType (pcl::SAC_RANSAC);
//    seg.setMaxIterations (1000);
//    seg.setDistanceThreshold (1.0);
//    int nr_points = (int) cloud_filtered->size ();
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
//    int i=0;
//    while (cloud_filtered->size () > 0.3 * nr_points)
//    {
//     // Segment the largest planar component from the remaining cloud
//         seg.setInputCloud (cloud_filtered);
//         seg.segment (*inliers, *coefficients);
//         if (inliers->indices.size () == 0)
//         {
//             std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
//             break;
//         }
//         // Extract the planar inliers from the input cloud
//         pcl::ExtractIndices<pcl::PointXYZ> extract;
//         extract.setInputCloud (cloud_filtered);
//         extract.setIndices (inliers);
//         extract.setNegative (false);
//         // Get the points associated with the planar surface
//         extract.filter (*cloud_plane);
//         if(cloud_plane->size()>500){
//             dl_plane.showCloud("cloud"+std::to_string(i),cloud_plane);
//             i+=1;
//             double time = cloud_filtered->size ()/cloud_plane->points.size();
//             std::cout<<"size"<<i<<" :"<<time<<std::endl;
//         }
//         std::cout << "PointCloud representing the planar component: " << cloud_plane->size () << " data points." << std::endl;
//         // Remove the planar inliers, extract the rest
//         extract.setNegative (true);
//         extract.filter (*cloud_f);
//         *cloud_filtered = *cloud_f;
//     }

    /*//////////////////////////////////////////////////////////////////////////////////////*/
    DL_PlaneFitting dl_planefitting;
    dl_planefitting.cloud_ = cloud;
    dl_planefitting.setIterationTime(1000);
    dl_planefitting.setDisThreshold(2.0);
    dl_planefitting.setMinRate(0.05);
    dl_planefitting.setStopRate(0.3);
    //dl_planefitting.ransacPlaneFitting();
    //平面拟合多线程版本
    dl_planefitting.core_ = 8;
    dl_planefitting.ransacPlaneFittingThread();
    return a.exec();
}
