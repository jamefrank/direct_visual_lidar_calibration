#include "file-utils.h"
#include <filesystem>
#include <iostream>
void FileUtils::readCornersFromYaml(const std::string& path, std::vector<cv::Point2f>& corners) {
  if (std::filesystem::exists(path)) return;
  corners.clear();
  cv::FileStorage fs(path, cv::FileStorage::READ);
  int pointNums;
  fs["pointNums"] >> pointNums;
  for (int i = 0; i < pointNums; i++) {
    std::string key = "p" + std::to_string(i);
    cv::Point2f corner;
    fs[key] >> corner;
    corners.emplace_back(corner);
  }
  fs.release();
}

Eigen::MatrixXd FileUtils::readCornersFromYaml(const std::string& path) {
  cv::FileStorage fs(path, cv::FileStorage::READ);

  if (!fs.isOpened()) {
    std::cerr << "Failed to open the file." << std::endl;
    return Eigen::MatrixXd();
  }

  cv::Mat pcd_corners;
  fs["corners"] >> pcd_corners;

  Eigen::MatrixXd tmp;
  cv::cv2eigen(pcd_corners, tmp);

  return tmp;
}
