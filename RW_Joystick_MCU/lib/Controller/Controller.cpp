#include "Controller.hpp"

namespace controller
{
  imu::Vector<3> QuaternionPD::Compute(const imu::Quaternion &q_desired,
                                       const imu::Quaternion &q_curr,
                                       const imu::Vector<3> &omega) const
  {
    imu::Quaternion error = CalcError(q_desired, q_curr); // Im surprised this even compiles with the different return types, but cool
    imu::Vector<3> xyz(error.x(), error.y(), error.z());
    return xyz * -kP - omega * kD;
  }

  imu::Quaternion QuaternionPD::CalcError(const imu::Quaternion &q_desired,
                                          const imu::Quaternion &q_curr) const
  {
    return q_desired.conjugate() * q_curr;
  }

  float WheelSpeedPD::Compute(const float rpm_desired,
                              const float rpm_curr,
                              const float dt)
  {
    float error = CalcError(rpm_desired, rpm_curr);
    float output = kP * error + (dt == 0 ? 0 : kD * (error - prev_err_) / dt);
    prev_err_ = error;
    return output;
  }

  float WheelSpeedPD::CalcError(const float rpm_desired, const float rpm_curr)
  {
    return rpm_desired - rpm_curr;
  }
} // namespace controller
