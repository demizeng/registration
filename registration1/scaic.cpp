#include <pcl/registration/ia_ransac.h>
#include <boost/make_shared.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/search/kdtree.h>
#include <pcl/point_representation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
using pcl::NormalEstimation;
using pcl::search::KdTree;
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

void visualize_pcd(PointCloud::Ptr pcd_src1,
   PointCloud::Ptr pcd_src2,
   PointCloud::Ptr pcd_final)
{
   //int vp_1, vp_2;
   // Create a PCLVisualizer object
   pcl::visualization::PCLVisualizer viewer("PCL Viewer");
   //viewer.createViewPort (0.0, 0, 0.5, 1.0, vp_1);
  // viewer.createViewPort (0.5, 0, 1.0, 1.0, vp_2);
   pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> src1_h (pcd_src1, 0, 255, 0);
   pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> src2_h (pcd_src2, 255, 0, 0);
   pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> final_h (pcd_final, 0, 0, 255);
   viewer.addPointCloud (pcd_src1, src1_h, "vp1_src1");
   viewer.addPointCloud (pcd_src2, src2_h, "vp1_src2");
   viewer.addPointCloud (pcd_final, final_h, "vp1_final");
   /*
   while(!viewer.wasStopped())
   {
       viewer.spinOnce();

   }
*/
   viewer.spin();
}

int
   main (int argc, char** argv)
{

   //PointCloud::Ptr cloud_final (new PointCloud);

   PointCloud::Ptr cloud_src_o (new PointCloud);//将长的转成短的//读取点云数据
   pcl::io::loadPCDFile ("bunny_rotated.pcd",*cloud_src_o);
   std::vector<int> indices_src; //保存去除的点的索引
   pcl::removeNaNFromPointCloud(*cloud_src_o,*cloud_src_o, indices_src); //去除点云中的NaN点
   std::cout<<"remove *cloud_src_o nan"<<endl;
   pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
   voxel_grid.setLeafSize(0.01,0.01,0.01);
   voxel_grid.setInputCloud(cloud_src_o);
   PointCloud::Ptr cloud_src (new PointCloud);
   voxel_grid.filter(*cloud_src);
   std::cout<<"down size *cloud_src_o from "<<cloud_src_o->size()<<"to"<<cloud_src->size()<<endl;
   pcl::io::savePCDFileASCII("bunny_src_down.pcd",*cloud_src);

   pcl::NormalEstimation<pcl::PointXYZ,pcl::Normal> ne_src;
   ne_src.setInputCloud(cloud_src);
   pcl::search::KdTree< pcl::PointXYZ>::Ptr tree_src(new pcl::search::KdTree< pcl::PointXYZ>());
   ne_src.setSearchMethod(tree_src);
   pcl::PointCloud<pcl::Normal>::Ptr cloud_src_normals(new pcl::PointCloud< pcl::Normal>);
   ne_src.setRadiusSearch(0.03);
   //ne_src.setKSearch(20);二选一
   ne_src.compute(*cloud_src_normals);

   PointCloud::Ptr cloud_tgt_o (new PointCloud);
   pcl::io::loadPCDFile ("bunny.pcd",*cloud_tgt_o);
   std::vector<int> indices_tgt; //保存去除的点的索引
   pcl::removeNaNFromPointCloud(*cloud_tgt_o,*cloud_tgt_o, indices_tgt); //去除点云中的NaN点
   std::cout<<"remove *cloud_tgt_o nan"<<endl;
   pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_2;
   voxel_grid_2.setLeafSize(0.01,0.01,0.01);
   voxel_grid_2.setInputCloud(cloud_tgt_o);
   PointCloud::Ptr cloud_tgt (new PointCloud);
   voxel_grid_2.filter(*cloud_tgt);
   std::cout<<"down size *cloud_tgt_o.pcd from "<<cloud_tgt_o->size()<<"to"<<cloud_tgt->size()<<endl;
   pcl::io::savePCDFileASCII("bunny_tgt_down.pcd",*cloud_tgt);


   pcl::NormalEstimation<pcl::PointXYZ,pcl::Normal> ne_tgt;
   ne_tgt.setInputCloud(cloud_tgt);
   pcl::search::KdTree< pcl::PointXYZ>::Ptr tree_tgt(new pcl::search::KdTree< pcl::PointXYZ>());
   ne_tgt.setSearchMethod(tree_tgt);
   pcl::PointCloud<pcl::Normal>::Ptr cloud_tgt_normals(new pcl::PointCloud< pcl::Normal>);
   //ne_tgt.setKSearch(20);
   ne_tgt.setRadiusSearch(0.03);
   //compute normal to cloud_normals
   ne_tgt.compute(*cloud_tgt_normals);


   //fpfh estimation
   pcl::FPFHEstimation<pcl::PointXYZ,pcl::Normal,pcl::FPFHSignature33> fpfh_src;
   fpfh_src.setInputCloud(cloud_src);
   fpfh_src.setInputNormals(cloud_src_normals);
   pcl::search::KdTree<PointT>::Ptr tree_src_fpfh (new pcl::search::KdTree<PointT>);
   fpfh_src.setSearchMethod(tree_src_fpfh);
   pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs_src(new pcl::PointCloud<pcl::FPFHSignature33>());
   fpfh_src.setRadiusSearch(0.05);
   //fpfh_src.setKSearch(20);
   fpfh_src.compute(*fpfhs_src);
   std::cout<<"compute *cloud_src fpfh"<<endl;

   pcl::FPFHEstimation<pcl::PointXYZ,pcl::Normal,pcl::FPFHSignature33> fpfh_tgt;
   fpfh_tgt.setInputCloud(cloud_tgt);
   fpfh_tgt.setInputNormals(cloud_tgt_normals);
   pcl::search::KdTree<PointT>::Ptr tree_tgt_fpfh (new pcl::search::KdTree<PointT>);
   fpfh_tgt.setSearchMethod(tree_tgt_fpfh);
   pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs_tgt(new pcl::PointCloud<pcl::FPFHSignature33>());
   fpfh_tgt.setRadiusSearch(0.05);
   //fpfh_tgt.setKSearch(20);
   fpfh_tgt.compute(*fpfhs_tgt);
   std::cout<<"compute *cloud_tgt fpfh"<<endl;

   //sample consensus initial alignment
   pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> scia;
   scia.setInputSource(cloud_src);
   scia.setInputTarget(cloud_tgt);
   scia.setSourceFeatures(fpfhs_src);
   scia.setTargetFeatures(fpfhs_tgt);
   //scia.setMinSampleDistance(1);
   //scia.setNumberOfSamples(2);
   //scia.setCorrespondenceRandomness(20);
   PointCloud::Ptr cloud_result_initial (new PointCloud);
   scia.align(*cloud_result_initial);
   std::cout  <<"totals time is:"<<clock()<<endl;
   std::cout<<scia.getFinalTransformation()<<endl;
   pcl::io::savePCDFileASCII("bunny_transformed_sac.pcd",*cloud_result_initial);
   visualize_pcd(cloud_src,cloud_tgt,cloud_result_initial);
return (0);
}
