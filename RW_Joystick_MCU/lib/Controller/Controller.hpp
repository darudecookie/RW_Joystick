#ifndef RW_SRC_CONTROLLER_HPP_
#define RW_SRC_CONTROLLER_HPP_

#include <../../include/quaternion.hpp>
#include <../../include/vector.hpp>

namespace controller
{
  // Quaternion error PD controller
  class QuaternionPD
  {
  public:
    QuaternionPD(const float p, const float d) : kP(p), kD(d) {}
    virtual ~QuaternionPD() = default;
    QuaternionPD(const QuaternionPD &other) = default;
    QuaternionPD &operator=(const QuaternionPD &other) = default;

    // Based on input parameters, calculates a torque requirement along the
    // x,y,z axes of rotation
    // given by the the control law u = -kP*q_error_xyz -kD*omega_body
    imu::Vector<3> Compute(const imu::Quaternion &q_desired,
                           const imu::Quaternion &q_curr, const imu::Vector<3> &omega) const;

  protected:
    // Calculates the quaternion error = (q*)q
    imu::Quaternion CalcError(const imu::Quaternion &setpoint,
                              const imu::Quaternion &curr_pos) const;

  private:
    const float kP, kD;
  };

  // Reaction wheel speed PD controller
  class WheelSpeedPD
  {
  public:
    WheelSpeedPD(const float p, const float d) : kP(p), kD(d), prev_err_(0) {}
    virtual ~WheelSpeedPD() = default;
    WheelSpeedPD(const WheelSpeedPD &other) = default;
    WheelSpeedPD &operator=(const WheelSpeedPD &other) = default;

    // Based on input parameters calculates a delta in pwm for a given wheel
    float Compute(const float rpm_desired, const float rpm_curr, const float dt);

  protected:
    // Calculates the error = rpm_desired - rpm_curr
    float CalcError(const float rpm_desired, const float rpm_curr);

  private:
    const float kP, kD;
    float prev_err_;
  };
} // namespace controller
#endif // RW_SRC_CONTROLLER_HPP_
