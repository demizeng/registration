#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

void visualize_pcd(PointCloud::Ptr pcd_src,
   PointCloud::Ptr pcd_tgt,
   PointCloud::Ptr pcd_final)
{
   //int vp_1, vp_2;
   // Create a PCLVisualizer object
   pcl::visualization::PCLVisualizer viewer("registration Viewer");
   //viewer.createViewPort (0.0, 0, 0.5, 1.0, vp_1);
  // viewer.createViewPort (0.5, 0, 1.0, 1.0, vp_2);
   pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> src_h (pcd_src, 0, 255, 0);
   pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tgt_h (pcd_tgt, 255, 0, 0);
   pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> final_h (pcd_final, 0, 0, 255);
   viewer.addPointCloud (pcd_src, src_h, "source cloud");
   viewer.addPointCloud (pcd_tgt, tgt_h, "tgt cloud");
   viewer.addPointCloud (pcd_final, final_h, "final cloud");
   //viewer.addCoordinateSystem(1.0);
   while (!viewer.wasStopped())
   {
       viewer.spinOnce(100);
       boost::this_thread::sleep(boost::posix_time::microseconds(100000));
   }
}

int main (int argc, char** argv)
{
    PointCloud::Ptr cloud_src (new PointCloud);
    PointCloud::Ptr cloud_tgt (new PointCloud);
    PointCloud::Ptr cloud_final(new PointCloud);
    pcl::io::loadPCDFile("bunny.pcd",*cloud_tgt);
    //pcl::io::loadPCDFile("bunny_transformed_sac.pcd",*cloud_src);
    pcl::io::loadPCDFile("bunny_rotated.pcd",*cloud_src);

    PointCloud::Ptr cloud_src_f(new PointCloud);
    pcl::VoxelGrid<PointT> voxel_filter;
    voxel_filter.setLeafSize(0.01, 0.01, 0.01);
    voxel_filter.setInputCloud(cloud_src);
    voxel_filter.filter(*cloud_src_f);
    std::cout << "Filtered cloud contains " << cloud_src_f->size()
        << " data points from bunny_rotated.pcd" << std::endl;

    //creates an instance of an IterativeClosestPoint and gives it some useful information
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(cloud_src);
    icp.setInputTarget(cloud_tgt);
    icp.setMaxCorrespondenceDistance (0.04);
    // Set the maximum number of iterations (criterion 1)
    icp.setMaximumIterations (50);
    // Set the transformation epsilon (criterion 2)
    icp.setTransformationEpsilon (1e-10);
    // Set the euclidean distance difference epsilon (criterion 3)
    icp.setEuclideanFitnessEpsilon (0.2);

    Eigen::Matrix4f init_guess;
    init_guess<<0.815905,0.569306,0.100948,-0.571533,
               -0.569183,0.821554,-0.0328553,0.399136,
               -0.101639,-0.030651,0.994349,0.0703133,
                0,0,0,1;

    //Call the registration algorithm which estimates the transformation and returns the transformed source (input) as output.
    icp.align(*cloud_final,init_guess);

    //Return the state of convergence after the last align run.
    //If the two PointClouds align correctly then icp.hasConverged() = 1 (true).
    std::cout << "has converged: " << icp.hasConverged() <<std::endl;

    //Obtain the Euclidean fitness score (e.g., sum of squared distances from the source to the target)
    std::cout << "score: " <<icp.getFitnessScore() << std::endl;

    //Get the final transformation matrix estimated by the registration method.
    std::cout << icp.getFinalTransformation() << std::endl;
    visualize_pcd(cloud_src,cloud_tgt,cloud_final);

    return (0);
}
