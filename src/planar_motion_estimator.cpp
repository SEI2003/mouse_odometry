#include "mouse_odometry/planar_motion_estimator.hpp"

#include <Eigen/QR>

#include <stdexcept>

namespace mouse_odometry
{

PlanarMotionEstimator::PlanarMotionEstimator(
  const SensorPosition & left,
  const SensorPosition & right)
{
  observation_matrix_ <<
    1.0, 0.0, -left.y,
    0.0, 1.0, left.x,
    1.0, 0.0, -right.y,
    0.0, 1.0, right.x;

  const auto decomposition = observation_matrix_.colPivHouseholderQr();
  if (decomposition.rank() < 3) {
    throw std::invalid_argument(
            "PMW3901 sensor geometry cannot determine planar 3-DoF motion");
  }
}

MotionEstimate PlanarMotionEstimator::estimate(
  const SensorDelta & left,
  const SensorDelta & right) const
{
  Eigen::Matrix<double, 4, 1> observations;
  observations << left.x, left.y, right.x, right.y;

  const Eigen::Vector3d solution =
    observation_matrix_.colPivHouseholderQr().solve(observations);
  const double residual =
    (observation_matrix_ * solution - observations).norm();

  return {solution.x(), solution.y(), solution.z(), residual};
}

const Eigen::Matrix<double, 4, 3> &
PlanarMotionEstimator::observationMatrix() const
{
  return observation_matrix_;
}

}  // namespace mouse_odometry
