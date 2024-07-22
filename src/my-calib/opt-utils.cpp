#include "opt-utils.h"

Eigen::Matrix3d OptUtils::calib_3d_to_2d_R(Eigen::MatrixX3d pcd_corners, Eigen::MatrixX2d img_corners, Eigen::Matrix3d K) {
  assert(pcd_corners.rows() >= 3);
  assert(img_corners.rows() >= 3);

  //
  int num_pcd = pcd_corners.rows();
  int num_img = img_corners.rows();

  //
  Eigen::MatrixX3d cam_normal_pts;
  cam_normal_pts.resize(num_img, 3);
  for (int i = 0; i < num_img; i++) {
    cam_normal_pts.row(i) << (img_corners(i, 0) - K(0, 2)) / K(0, 0), (img_corners(i, 1) - K(1, 2)) / K(1, 1), 1.0;
  }
  cam_normal_pts.rowwise().normalize();

  //
  pcd_corners.rowwise().normalize();

  //
  const Eigen::Matrix3d AB = cam_normal_pts.transpose() * pcd_corners;
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(AB, Eigen::ComputeFullU | Eigen::ComputeFullV);
  const Eigen::Matrix3d U = svd.matrixU();
  const Eigen::Matrix3d V = svd.matrixV();
  const Eigen::Matrix3d D = svd.singularValues().asDiagonal();
  Eigen::Matrix3d S = Eigen::Matrix3d::Identity();

  double det = U.determinant() * V.determinant();
  if (det < 0.0) {
    S(2, 2) = -1.0;
  }

  const Eigen::Matrix3d R_camera_lidar = U * S * V.transpose();

  return R_camera_lidar;
}

Eigen::Matrix4d OptUtils::calib_3d_to_2d_T(Eigen::MatrixX3d pcd_corners, Eigen::MatrixX2d img_corners, Eigen::Matrix3d K) {
  //
  auto R = calib_3d_to_2d_R(pcd_corners, img_corners, K);
  Eigen::Quaterniond rotation(R);
  Eigen::Vector3d translation(0.0, -1.0, -1.0);
  std::cout << "Init Rotation: " << rotation.toRotationMatrix() << std::endl;
  std::cout << "Init Translation: " << translation.transpose() << std::endl;

  //
  ceres::Problem problem;
  double rotation_params[] = {rotation.w(), rotation.x(), rotation.y(), rotation.z()};
  ceres::LocalParameterization* quaternion_parameterization = new ceres::EigenQuaternionParameterization();
  problem.AddParameterBlock(rotation_params, 4, quaternion_parameterization);
  // problem.SetParameterBlockConstant(rotation_params);  // fix

  double translation_params[] = {translation.x(), translation.y(), translation.z()};
  problem.AddParameterBlock(translation_params, 3);

  // problem.SetParameterLowerBound(translation_params, 0, -2.0);
  // problem.SetParameterUpperBound(translation_params, 0, 2.0);
  // problem.SetParameterLowerBound(translation_params, 1, -2.0);
  // problem.SetParameterUpperBound(translation_params, 1, 0.0);
  // problem.SetParameterLowerBound(translation_params, 2, -2.0);
  // problem.SetParameterUpperBound(translation_params, 2, 0.0);

  for (int i = 0; i < pcd_corners.rows(); i++) {
    ceres::CostFunction* cost_function = PinholeReprojectionError::Create(pcd_corners.row(i), img_corners.row(i), K);
    problem.AddResidualBlock(cost_function, nullptr, rotation_params, translation_params);
  }

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::LinearSolverType::DENSE_SCHUR;
  options.minimizer_progress_to_stdout = true;
  options.max_num_iterations = 10000;
  options.gradient_check_relative_precision = 1e-6;
  options.gradient_check_numeric_derivative_relative_step_size = 1e-6;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.BriefReport() << std::endl;
  //
  Eigen::Quaterniond optimized_rotation(rotation_params[0], rotation_params[1], rotation_params[2], rotation_params[3]);
  Eigen::Vector3d optimized_translation(translation_params[0], translation_params[1], translation_params[2]);
  std::cout << "Optimized Rotation: " << optimized_rotation.toRotationMatrix() << std::endl;
  std::cout << "Optimized Translation: " << optimized_translation.transpose() << std::endl;

  //
  Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();
  transformation_matrix.topLeftCorner<3, 3>() = optimized_rotation.toRotationMatrix();
  transformation_matrix.topRightCorner<3, 1>() = optimized_translation;
  return transformation_matrix;
}
