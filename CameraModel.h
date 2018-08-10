//
// Created by hy1703 on 8/9/18.
//

#ifndef CAMERAMODEL_H
#define CAMERAMODEL_H
#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>

using namespace std;

class CameraModel {
public:
    void setIntrinsicParameters(float fx, float fy, float cx, float cy,
                                float k[], float alpha = 0);
    void setExtrinsicParameters(Eigen::Matrix3f R, Eigen::Vector3f T);
    void fromWorldToImage(Eigen::Vector3f worldPt, Eigen::Vector2f& imagePt);
    void fromWorldToImage(float x, float y, float z, float& u, float& v);
private:
    Eigen::Matrix3f _R;
    Eigen::Vector3f _T;
    float _fx;
    float _fy;
    float _cx;
    float _cy;
    float _k[5];
    float _alpha;
};

#endif