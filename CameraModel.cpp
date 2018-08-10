//
// Created by hy1703 on 8/9/18.
//

#include "CameraModel.h"

void CameraModel::setIntrinsicParameters(float fx, float fy, float cx, float cy,
                            float k[], float alpha)
{
    _fx = fx;
    _fy = fy;
    _cx = cx;
    _cy = cy;
    _alpha = alpha;
    for(int i = 0;i < 5;i++)
        _k[i] = k[i];
}

void CameraModel::setExtrinsicParameters(Eigen::Matrix3f R, Eigen::Vector3f T)
{
    _R = R;
    _T = T;
}

#include <iostream>

void CameraModel::fromWorldToImage(Eigen::Vector3f worldPt, Eigen::Vector2f& imagePt)
{
    Eigen::Vector3f cameraPt;
    cameraPt = _R * worldPt;
    cameraPt = cameraPt + _T;
    cameraPt /= cameraPt(2, 0);
    //std::cout << "cameraPt: " << cameraPt << std::endl;
    float r2 = cameraPt(0, 0) * cameraPt(0, 0) + cameraPt(1, 0) * cameraPt(1, 0);
    float r4 = r2 * r2;
    float r6 = r2 * r2 * r2;
    float cdist = 1 + _k[0] * r2 + _k[1] * r4 + _k[4] * r6;
    //std::cout << "cdist: " << cdist << std::endl;
    Eigen::Vector2f xd1(cameraPt(0, 0) * cdist, cameraPt(1, 0) * cdist);
    float a1 = 2 * cameraPt(0, 0) * cameraPt(1, 0);
    float a2 = r2 + 2 * cameraPt(0, 0) * cameraPt(0, 0);
    float a3 = r2 + 2 * cameraPt(1, 0) * cameraPt(1, 0);
   // std::cout << a1 << "," << a2 << "," << a3 << std::endl;
    Eigen::Vector2f delta_x(_k[2] * a1 + _k[3] * a2, _k[2] * a3 + _k[3] * a1);
    Eigen::Vector2f xd2 = xd1 + delta_x;
    xd2(0, 0) = xd2(0, 0) + _alpha * xd2(1, 0);
    imagePt(0, 0) = xd2(0, 0) * _fx + _cx;
    imagePt(1, 0) = xd2(1, 0) * _fy + _cy;
}

void CameraModel::fromWorldToImage(float x, float y, float z, float& u, float& v)
{
    Eigen::Vector3f worldPt(x, y, z);
    Eigen::Vector2f imagePt;
    fromWorldToImage(worldPt, imagePt);
    u = imagePt(0, 0);
    v = imagePt(1, 0);
}