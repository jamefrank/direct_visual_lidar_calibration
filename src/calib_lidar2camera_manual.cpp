#include <iostream>

#include <filesystem>

#include <boost/program_options.hpp>
#include <boost/algorithm/string.hpp>

#include <pcl/io/pcd_io.h>

#include "file-utils.h"
#include "opt-utils.h"

int main(int argc, char** argv) {
  using namespace boost::program_options;
  options_description description("calib_lidar2camera_manual");

  // clang-format off
  description.add_options()
    ("help", "produce help message")
    ("pcd_yaml", value<std::string>(), "directory of pcd corners yaml")
    ("img_yaml", value<std::string>(), "directory of img corners yaml")
    ("visualize,v", "if true, show extracted images and points")
  ;
  // clang-format on
  positional_options_description p;
  p.add("pcd_yaml", 1);
  p.add("img_yaml", 1);

  variables_map vm;
  store(command_line_parser(argc, argv).options(description).positional(p).run(), vm);
  notify(vm);

  if (vm.count("help") || !vm.count("pcd_yaml") || !vm.count("img_yaml")) {
    std::cout << description << std::endl;
    return true;
  }

  const std::string pcd_yaml = vm["pcd_yaml"].as<std::string>();
  const std::string img_yaml = vm["img_yaml"].as<std::string>();
  std::cout << "pcd_yaml: " << pcd_yaml << std::endl;
  std::cout << "img_yaml: " << img_yaml << std::endl;
  // std::filesystem::create_directories(dst_dir);

  auto pcd_corners = FileUtils::readCornersFromYaml(pcd_yaml);
  auto img_corners = FileUtils::readCornersFromYaml(img_yaml);
  std::cout << "pcd_corners: " << pcd_corners << std::endl;
  std::cout << "img_corners: " << img_corners << std::endl;

  //
  Eigen::Matrix3d K;
  // clang-format off
  // lb
  // K << 1.22394814e+03, 0.00000000e+00, 9.83941168e+02, 
  //      0.00000000e+00, 1.22409659e+03, 5.05492343e+02, 
  //      0.00000000e+00, 0.00000000e+00, 1.00000000e+00;
  // lf
  // K << 1.17956159e+03, 0.00000000e+00, 9.62218998e+02,
  //      0.00000000e+00, 1.18011466e+03, 5.03656666e+02,
  //      0.00000000e+00, 0.00000000e+00, 1.00000000e+00;
  // rf
  // K << 1.22394814e+03, 0.00000000e+00, 9.83941168e+02,
      //  0.00000000e+00, 1.22409659e+03, 5.05492343e+02,
      //  0.00000000e+00, 0.00000000e+00, 1.00000000e+00;
  // //rb
  // K << 1.17956159e+03, 0.00000000e+00, 9.62218998e+02,
  //      0.00000000e+00, 1.18011466e+03, 5.03656666e+02,
  //      0.00000000e+00, 0.00000000e+00, 1.00000000e+00;

  //b
  K <<1.19125616e+03, 0.00000000e+00, 1.01093900e+03,
      0.00000000e+00, 1.19221473e+03, 5.28980999e+02,
      0.00000000e+00, 0.00000000e+00, 1.00000000e+00;

  // clang-format on
  auto T_3d_2d = OptUtils::calib_3d_to_2d_T(pcd_corners, img_corners, K);

  return 0;
}
