/*
 * @Author: fanjin
 * @Date: 2024-07-21 03:06:47
 * @LastEditors: fanjin
 * @LastEditTime: 2024-07-21 06:04:20
 * @FilePath: /ros2_ws/src/direct_visual_lidar_calibration/include/my-calib/opt-utils.h
 * @Description: 优化求解工具
 *
 * Copyright (c) 2024 by Frank, All Rights Reserved.
 */

#pragma once

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include <Eigen/Core>
#include <Eigen/Eigen>

namespace OptUtils {
Eigen::Matrix3d calib_3d_to_2d_R(Eigen::MatrixX3d pcd_corners, Eigen::MatrixX2d img_corners, Eigen::Matrix3d K);
Eigen::Matrix4d calib_3d_to_2d_T(Eigen::MatrixX3d pcd_corners, Eigen::MatrixX2d img_corners, Eigen::Matrix3d K);

//

};  // namespace OptUtils

//
class PinholeReprojectionError {
public:
  PinholeReprojectionError(Eigen::Vector3d pt_world, Eigen::Vector2d pt_img, Eigen::Matrix3d K) : pt_world_(pt_world), pt_img_(pt_img), K_(K){};

  template <typename T>
  bool operator()(const T* const rotation, const T* const translation, T* residuals) const {
    //
    Eigen::Quaternion<T> q(rotation[0], rotation[1], rotation[2], rotation[3]);
    Eigen::Matrix<T, 3, 1> t(translation[0], translation[1], translation[2]);
    Eigen::Matrix<T, 3, 1> p_world(pt_world_.cast<T>());
    Eigen::Matrix<T, 2, 1> p_img(pt_img_.cast<T>());
    Eigen::Matrix<T, 3, 3> K(K_.cast<T>());
    //
    Eigen::Matrix<T, 3, 1> p_cam_normal = K * (q.toRotationMatrix() * p_world + t);

    //
    Eigen::Matrix<T, 2, 1> p_img_reprojected(p_cam_normal(0) / p_cam_normal(2), p_cam_normal(1) / p_cam_normal(2));

    //
    residuals[0] = p_img_reprojected(0) - p_img(0);
    residuals[1] = p_img_reprojected(1) - p_img(1);

    return true;
  }

  static ceres::CostFunction* Create(Eigen::Vector3d pt_world, Eigen::Vector2d pt_img, Eigen::Matrix3d K) {
    return (new ceres::AutoDiffCostFunction<PinholeReprojectionError, 2, 4, 3>(new PinholeReprojectionError(pt_world, pt_img, K)));
  }

  Eigen::Vector3d pt_world_;
  Eigen::Vector2d pt_img_;
  Eigen::Matrix3d K_;
};