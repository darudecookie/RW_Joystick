#include "RwStatus.hpp"

namespace rw_status {
void RwStatus::UpdateStatus() {
  // TODO need more info on pins
}

uint8_t RwStatus::NumWorkingWheels() {
  // TODO check if wheel_locked_ is equivalent
  return wheel_status_[0] + wheel_status_[1]
    + wheel_status_[2] + wheel_status_[3];
}

float RwStatus::GetWheelSpeed(const uint8_t wheel_num) {
  return interrupt::wheel_rpm[wheel_num];
}
}  // namespace rw_status

// namespace interrupt {
// void TimerHandler() {
//   global_time++;
// }

// void ReadRpm0() {
//   wheel_rpm[0] = (kSecPerMin*kRotPerFG*kGlobalRate
//     /(float) (global_time - prev_time[0])) - kZeroRpm;
//   prev_time[0] = global_time;
// }
// void ReadRpm1() {
//   wheel_rpm[1] = (kSecPerMin*kRotPerFG*kGlobalRate
//     /(float) (global_time - prev_time[1])) - kZeroRpm;
//   prev_time[1] = global_time;
// }
// void ReadRpm2() {
//   wheel_rpm[2] = (kSecPerMin*kRotPerFG*kGlobalRate
//     /(float) (global_time - prev_time[2])) - kZeroRpm;
//   prev_time[2] = global_time;
// }
// void ReadRpm3() {
//   wheel_rpm[3] = (kSecPerMin*kRotPerFG*kGlobalRate
//     /(float) (global_time - prev_time[3])) - kZeroRpm;
//   prev_time[3] = global_time;
// }
// }  // namespace interrupt
