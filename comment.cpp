#include "comment.h"
#include <stdlib.h>
#include <stdio.h>
#include <time.h>

std::vector<std::string> scanfolder(std::string folder, const char* ext)
{
  std::vector<std::string> file_list;
  DIR * dir = opendir(folder.c_str());
  struct dirent * ptr;
  while ((ptr = readdir(dir)) != NULL)
  {
    if (strcmp(ptr->d_name, ".") == 0 || strcmp(ptr->d_name,"..") == 0)
    {
      continue;
    }
    if (strcmp(get_extent_from_path(ptr->d_name).c_str(), ext) == 0)
    {
       file_list.push_back(folder + "/" + ptr->d_name);
    }
  }
  sort(file_list.begin(), file_list.end());
  return file_list;
}

std::string get_filename_from_path(std::string str_path)
{
  int p1 = str_path.find_last_of('/') + 1;
  int p2 = str_path.find_last_of('.');
  return str_path.substr(p1, p2 - p1);
}

std::string get_extent_from_path(std::string str_path)
{
  int p1 = str_path.find_last_of('.');
  std::string ext = str_path.substr(p1 + 1, str_path.length() - p1);
  return ext;
}

void cloud_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, ConditionFunc condition)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp (new pcl::PointCloud<pcl::PointXYZ>);
  int i = 0;
  (*cloud_temp) += (*cloud);
  cloud->clear();
  std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ> >::iterator it = cloud_temp->points.begin();
  while(it!=cloud_temp->points.end())
  {
    i++;
    if(!(condition(*it)))
    {
      it++;
      continue;
    }
    //it->z =0;
    cloud->points.push_back(*it);
    it++;
  }
  cloud->width = cloud->points.size();
  cloud->height = 1;
  cloud->is_dense = false;
}


void get_rotation_matrix2d(float angle, float* m)
{
    m[0] = cos(angle);
    m[1] = -sin(angle);
    m[2] = sin(angle);
    m[3] = cos(angle);
}

void downsample(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, float rate)
{
  srand(time(0));
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp (new pcl::PointCloud<pcl::PointXYZ>);
  int i = 0;
  (*cloud_temp) += (*cloud);
  cloud->clear();
  for(int i = 0;i<cloud_temp->points.size();i++)
  {
     float r = float(rand_between(0, 100)) / 100.0;
     if(r < rate)
     {
       pcl::PointXYZ pt = cloud_temp->points[i];
       cloud->points.push_back(pt);
     }
  }
  cloud->width = cloud->points.size();
  cloud->height = 1;
  cloud->is_dense = false;
}

void get_normal_of_plane(pcl::ModelCoefficients coefficients, pcl::Normal& normal)
{
  std::vector<float>& coef = coefficients.values;
  normal.normal_x = coef[0];
  normal.normal_y = coef[1];
  normal.normal_z = coef[2];
  float nn = sqrtf(normal.normal_x * normal.normal_x + normal.normal_y * normal.normal_y + normal.normal_z * normal.normal_z);
  normal.normal_x /= nn;
  normal.normal_y /= nn;
  normal.normal_z /= nn; 
}

void plane_detect(pcl::PointCloud<pcl::PointXYZ>::Ptr& origin_cloud, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& vcPlanes, 
        std::vector<pcl::ModelCoefficients::Ptr>& vcCoefficients, int maxPlaneCount)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr filter_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    *cloud = *origin_cloud;
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(300);
    seg.setDistanceThreshold(0.2);

    for(int i = 0;i<maxPlaneCount;i++)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coefficients);
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud);
        extract.setIndices(inliers);
        extract.filter(*cloud_plane);

        extract.setNegative(true);
        extract.filter(*filter_cloud);
        *cloud = *filter_cloud;
        vcPlanes.push_back(cloud_plane);
        vcCoefficients.push_back(coefficients);
    }
}

pcl::PointXYZ get_juntion_of_3planes(pcl::ModelCoefficients c1, pcl::ModelCoefficients c2, pcl::ModelCoefficients c3)
{
  Eigen::Matrix3d A;
  A << c1.values[0],c1.values[1],c1.values[2], 
       c2.values[0],c2.values[1],c2.values[2], 
       c3.values[0],c3.values[1],c3.values[2]; 
  Eigen::Vector3d b(-c1.values[3],-c2.values[3],-c3.values[3]);
  Eigen::Vector3d x = A.colPivHouseholderQr().solve(b);
  pcl::PointXYZ pt;
  pt.x = x[0];
  pt.y = x[1];
  pt.z = x[2];
  return pt;
}


double get_angle_of_vectors(pcl::Normal n1, pcl::Normal n2)
{
  double a = n1.normal_x * n2.normal_x + n1.normal_y * n2.normal_y + n1.normal_z * n2.normal_z;
  double s1 = sqrtf(n1.normal_x * n1.normal_x + n1.normal_y * n1.normal_y + n1.normal_z * n1.normal_z);
  double s2 = sqrtf(n2.normal_x * n2.normal_x + n2.normal_y * n2.normal_y + n2.normal_z * n2.normal_z);
  double angle = acos(a / (s1 * s2)) * 180.0 / 3.1415;
  return angle;
}

void get_pose_of_lidar(pcl::Normal ns10, pcl::Normal ns20,
                       pcl::Normal ns11, pcl::Normal ns21,
                       pcl::Normal ns12, pcl::Normal ns22, 
                       pcl::PointXYZ rp1, pcl::PointXYZ rp2,
                       double* matrix)
{
  float a1x = ns20.normal_x + ns10.normal_x;
  float a1y = ns20.normal_y + ns10.normal_y;
  float a1z = ns20.normal_z + ns10.normal_z;

  float a2x = ns21.normal_x + ns11.normal_x;
  float a2y = ns21.normal_y + ns11.normal_y;
  float a2z = ns21.normal_z + ns11.normal_z;

  float a3x = ns22.normal_x + ns12.normal_x;
  float a3y = ns22.normal_y + ns12.normal_y;
  float a3z = ns22.normal_z + ns12.normal_z;

  float d1x = ns20.normal_x - ns10.normal_x;
  float d1y = ns20.normal_y - ns10.normal_y;
  float d1z = ns20.normal_z - ns10.normal_z;

  float d2x = ns21.normal_x - ns11.normal_x;
  float d2y = ns21.normal_y - ns11.normal_y;
  float d2z = ns21.normal_z - ns11.normal_z;

  float d3x = ns22.normal_x - ns12.normal_x;
  float d3y = ns22.normal_y - ns12.normal_y;
  float d3z = ns22.normal_z - ns12.normal_z;
  Eigen::Matrix<double, 9, 3> A;
  A << 0, a1z, -a1y, -a1z, 0, a1x, a1y, -a1x, 0,
       0, a2z, -a2y, -a2z, 0, a2x, a2y, -a2x, 0,
       0, a3z, -a3y, -a3z, 0, a3x, a3y, -a3x, 0;
  Eigen::Matrix<double, 9, 1> D;
  D << d1x, d1y, d1z, d2x, d2y, d2z, d3x, d3y, d3z;
  // solve
  Eigen::Vector3d u = (A.transpose() * A).inverse() * A.transpose() * D;
  //cout << "u = " << u << endl;
  Eigen::Matrix3d S;
  S << 0, -u[2], u[1], u[2], 0, -u[0], -u[1], u[0], 0;
  //cout << S << endl;
  Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d R = (S + I) * (I - S).inverse();
  //std::cout << "R = " << R << std::endl;
  //T
  Eigen::Vector3d v1(rp1.x, rp1.y, rp1.z);
  Eigen::Vector3d v2(rp2.x, rp2.y, rp2.z);
  //Eigen::Vector3d T = v1 - R * v2;
  Eigen::Vector3d T = v2 - R * v1;
  //std::cout << "T = " << T << std::endl;

  matrix[0] = R(0, 0);
  matrix[1] = R(0, 1);
  matrix[2] = R(0, 2);
  matrix[3] = T(0, 0);
  matrix[4] = R(1, 0);
  matrix[5] = R(1, 1);
  matrix[6] = R(1, 2);
  matrix[7] = T(1, 0);
  matrix[8] = R(2, 0);
  matrix[9] = R(2, 1);
  matrix[10] = R(2, 2);
  matrix[11] = T(2, 0);
  matrix[12] = 0;
  matrix[13] = 0;
  matrix[14] = 0;
  matrix[15] = 1;
}


void load_matrix(std::string filename, std::vector<Eigen::Matrix<double, 4, 4> >& matrixs)
{
  matrixs.clear();
  FILE *fp = fopen(filename.c_str(), "r");
  int i = 0;
  while(1)
  {
    double m[16];
    if(16 != fscanf(fp, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n", 
                                &(m[0]), &(m[1]), &(m[2]), &(m[3]), 
                                &(m[4]), &(m[5]), &(m[6]), &(m[7]),  
                                &(m[8]), &(m[9]), &(m[10]), &(m[11]), 
                                &(m[12]), &(m[13]), &(m[14]), &(m[15])))
                                break;
    Eigen::Matrix<double, 4, 4> matrix;
    matrix << m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8], m[9], m[10], m[11], m[12], m[13], m[14], m[15];
    matrixs.push_back(matrix);
  }
  fclose(fp);
}

void imu_lidar_calibration(std::vector<Eigen::Matrix<double, 4, 4> >imu_motion, 
                           std::vector<Eigen::Matrix<double, 4, 4> > lidar_motion,
                           Eigen::Matrix3d& R, Eigen::Vector3d& t)
{
  int count = imu_motion.size();
  Eigen::Matrix3d M, R_X;
  M.setZero(3, 3);
  std::vector<Eigen::Matrix3d> R_A;
  std::vector<Eigen::Vector3d> t_A, t_B;
  for(int i = 0;i<count;i++)
  {
      Eigen::Matrix3d lidar_Rotate = lidar_motion[i].topLeftCorner(3, 3);
      Eigen::Vector3d lidar_Trans = lidar_motion[i].block(0, 3, 3, 1);
      Eigen::Matrix3d imu_Rotate = imu_motion[i].topLeftCorner(3, 3);
      Eigen::Vector3d imu_Trans = imu_motion[i].block(0, 3, 3, 1);

      R_A.push_back(lidar_Rotate);
      t_A.push_back(lidar_Trans);
      t_B.push_back(imu_Trans);

      Sophus::SO3 SO3_R_Lidar(lidar_Rotate);
      Sophus::SO3 SO3_R_IMU(imu_Rotate);
      Eigen::Vector3d alpha = SO3_R_Lidar.log();
      Eigen::Vector3d beta = SO3_R_IMU.log();
      M = M + beta*alpha.transpose();
  }
  Eigen::Matrix3d M2 = M.transpose()*M;
  R_X = (M2.pow(-0.5))*(M.transpose());
  Eigen::MatrixXd C(3*count, 3);
  Eigen::VectorXd d(3*count);
  Eigen::Matrix3d I;
  I.setIdentity();
  for(int i = 0;i<count;i++)
  {
      C.block(i*3, 0, 3, 3) = I - R_A[i];
      d.block(i*3, 0, 3, 1) = t_A[i] - R_X*t_B[i];
  }
  Eigen::Vector3d t_X = (C.transpose()*C).inverse()*(C.transpose()*d);
  R = R_X;
  t = t_X;
}