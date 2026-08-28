#ifndef MOUSE_ODOMETRY__PLANAR_MOTION_ESTIMATOR_HPP_
#define MOUSE_ODOMETRY__PLANAR_MOTION_ESTIMATOR_HPP_

#include <Eigen/Core>

namespace mouse_odometry
{

struct SensorPosition
{
  double x;
  double y;
};

struct SensorDelta
{
  double x;
  double y;
};

struct MotionEstimate
{
  double dx;
  double dy;
  double dyaw;
  double residual;
};

class PlanarMotionEstimator
{
public:
  PlanarMotionEstimator(
    const SensorPosition & left,
    const SensorPosition & right);

  MotionEstimate estimate(
    const SensorDelta & left,
    const SensorDelta & right) const;

  const Eigen::Matrix<double, 4, 3> & observationMatrix() const;

private:
  Eigen::Matrix<double, 4, 3> observation_matrix_;
};

}  // namespace mouse_odometry

#endif  // MOUSE_ODOMETRY__PLANAR_MOTION_ESTIMATOR_HPP_
