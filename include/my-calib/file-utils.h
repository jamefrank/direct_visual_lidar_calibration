/*
 * @Author: fanjin
 * @Date: 2024-07-21 02:46:44
 * @LastEditors: fanjin
 * @LastEditTime: 2024-07-21 03:31:15
 * @FilePath: /ros2_ws/src/direct_visual_lidar_calibration/include/my-calib/file-utils.h
 * @Description:  读写文件的工具
 *
 * Copyright (c) 2024 by Frank, All Rights Reserved.
 */

#pragma once

#include <Eigen/Core>
#include <Eigen/Eigen>

#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>

namespace FileUtils {
void readCornersFromYaml(const std::string& path, std::vector<cv::Point2f>& corners);
Eigen::MatrixXd readCornersFromYaml(const std::string& path);
};  // namespace FileUtils