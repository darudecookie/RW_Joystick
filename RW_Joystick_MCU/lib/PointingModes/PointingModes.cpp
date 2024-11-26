#include "PointingModes.hpp"
#include "Controller.hpp"

#define CLAMP(x, low, high) ({\
  __typeof__(x) __x = (x); \
  __typeof__(low) __low = (low);\
  __typeof__(high) __high = (high);\
  __x > __high ? __high : (__x < __low ? __low : __x);\
  })

namespace pointing_modes {
FourWheelMode::FourWheelMode() {
  // set the values for the pseudoinverse, as described in the doc
  pseudoinverse_.cell(0,0) = kSqrt3Div4;
  pseudoinverse_.cell(1,0) = -kSqrt3Div4;
  pseudoinverse_.cell(2,0) = -kSqrt3Div4;
  pseudoinverse_.cell(3,0) = kSqrt3Div4;

  pseudoinverse_.cell(0,1) = -kSqrt3Div4;
  pseudoinverse_.cell(1,1) = kSqrt3Div4;
  pseudoinverse_.cell(2,1) = -kSqrt3Div4;
  pseudoinverse_.cell(3,1) = kSqrt3Div4;

  pseudoinverse_.cell(0,2) = kSqrt3Div4;
  pseudoinverse_.cell(1,2) = kSqrt3Div4;
  pseudoinverse_.cell(2,2) = kSqrt3Div4;
  pseudoinverse_.cell(3,2) = kSqrt3Div4;

  pseudoinverse_.cell(0,3) = 0;
  pseudoinverse_.cell(1,3) = 0;
  pseudoinverse_.cell(2,3) = 0;
  pseudoinverse_.cell(3,3) = 0;
}

void FourWheelMode::Calculate(const imu::Vector<3>& sat_torque, float wheel_torques[]) const {
  imu::Vector<4> v(-sat_torque[0], -sat_torque[1], -sat_torque[2], 0);
  for (uint8_t i = 0; i < kNumWheels; i++) {
    wheel_torques[i] = pseudoinverse_.row_to_vector(i).dot(v);  // TODO incorporate the other part of the torque equation
  }
}

void FourWheelMode::Pid_Speed(const float wheel_torques[], const uint32_t dt,
  controller::WheelSpeedPD& wpd, volatile float wheel_rpm[], uint8_t pwm[]) {
  for (uint8_t i = 0; i < kNumWheels; i++) {
    // integrate (torques / wheel_moment_of_inertia) to get speed
    float delta_rpm = (wheel_torques[i] / kWheelMoment[i] * dt / 1000.0) * radps_rpm;
    wheel_pwm_[i] = CLAMP(wheel_pwm_[i] + wpd.Compute(wheel_rpm[i] + delta_rpm, wheel_rpm[i], dt), 0.0, 127.0);
    pwm[i] = wheel_pwm_[i];
  }
}

void FourWheelMode::Test_Speed_Command(const float desired_rpm[], volatile float wheel_rpm[], const uint32_t dt,
  controller::WheelSpeedPD& wpd, uint8_t pwm[]) {
  for (uint8_t i = 0; i < kNumWheels; i++) {
    // integrate (torques / wheelmoment) to get speed
    wheel_pwm_[i] = CLAMP(wheel_pwm_[i] + wpd.Compute(desired_rpm[i], wheel_rpm[i], dt), 0.0, 127.0);
    pwm[i] = wheel_pwm_[i];
  }
}

}  // namespace pointing_modes