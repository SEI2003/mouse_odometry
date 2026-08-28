#include "mouse_odometry/planar_motion_estimator.hpp"

#include <Eigen/QR>

#include <cmath>
#include <stdexcept>

namespace mouse_odometry
{

SensorDelta convertSensorDeltaToBody(
  double raw_dx,
  double raw_dy,
  double x_meter_per_count,
  double y_meter_per_count,
  double sensor_yaw)
{
  const double sensor_x = raw_dx * x_meter_per_count;
  const double sensor_y = raw_dy * y_meter_per_count;
  const double cosine = std::cos(sensor_yaw);
  const double sine = std::sin(sensor_yaw);
  return {
    cosine * sensor_x - sine * sensor_y,
    sine * sensor_x + cosine * sensor_y};
}

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
  // This residual only measures consistency with the observable rigid-motion
  // subspace. In a two-sensor layout, certain one-sided X errors are exactly
  // explainable as translation plus yaw and therefore have zero residual.
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
