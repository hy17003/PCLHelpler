#ifndef COMMENT_H
#define COMMENT_H

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <sophus/so3.h>
#include <unsupported/Eigen/MatrixFunctions>


#define rand_between(a, b) ((rand() % (b-a+1))+ a)
typedef bool(*ConditionFunc)(pcl::PointXYZ);

std::vector<std::string> scanfolder(std::string folder, const char* ext);
std::string get_filename_from_path(std::string str_path);
std::string get_extent_from_path(std::string str_path);
void cloud_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, ConditionFunc condition);
void get_rotation_matrix2d(float angle, float* m);
void downsample(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, float rate = 0.5);
void get_normal_of_plane(pcl::ModelCoefficients coefficients, pcl::Normal& normal);
void plane_detect(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& planes_cloud, 
        std::vector<pcl::ModelCoefficients::Ptr>& coefficients, int maxPlaneCount);
pcl::PointXYZ get_juntion_of_3planes(pcl::ModelCoefficients c1, pcl::ModelCoefficients c2, pcl::ModelCoefficients c3);
double get_angle_of_vectors(pcl::Normal n1, pcl::Normal n2);
void get_pose_of_lidar(pcl::Normal ns10, pcl::Normal ns20,
                       pcl::Normal ns11, pcl::Normal ns21,
                       pcl::Normal ns12, pcl::Normal ns22, 
                       pcl::PointXYZ rp1, pcl::PointXYZ rp2,
                       double* matrix);
void load_matrix(std::string filename, std::vector<Eigen::Matrix<double, 4, 4> >& matrixs);
void imu_lidar_calibration(std::vector<Eigen::Matrix<double, 4, 4> >imu_motion, 
                           std::vector<Eigen::Matrix<double, 4, 4> > lidar_motion,
                           Eigen::Matrix3d& R, Eigen::Vector3d& t);
#endif