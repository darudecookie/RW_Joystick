#ifndef RW_SRC_RWSTATUSCHECKER_HPP_
#define RW_SRC_RWSTATUSCHECKER_HPP_

#include <stdint.h>


namespace rw_status
{
  static constexpr uint8_t kNumWheels = 4;

  // A class which tracks the state of the reaction wheel system.
  class RwStatus
  {
  public:
    // init constructor
    RwStatus() : wheel_status_{1, 1, 1, 1}, wheel_locked_{0, 0, 0, 0},
                 wheel_dir_{1, 1, 1, 1} {}
    ~RwStatus() = default;
    // why would you need these?
    RwStatus(const RwStatus &other) = delete;
    RwStatus &operator=(const RwStatus &other) = delete;

    // Updates internal state of this based on physical data
    void UpdateStatus();

    // returns the number of unbroken wheels. Bounded [0,4]
    uint8_t NumWorkingWheels();

    // returns the wheel speed of wheel_num in TODO units
    float GetWheelSpeed(const uint8_t wheel_num);

  private:
    uint8_t wheel_status_[4]; // 1 if alive, 0 otherwise
    uint8_t wheel_locked_[4]; // 1 if locked, 0 otherwise
    uint8_t wheel_dir_[4];    // HIGH or LOW
  };
} // namespace rw_status

/*
These define's must be placed at the beginning before #include "TimerInterrupt.h"
_TIMERINTERRUPT_LOGLEVEL_ from 0 to 4
Don't define _TIMERINTERRUPT_LOGLEVEL_ > 0. Only for special ISR debugging only. Can hang the system.
*/
#define TIMER_INTERRUPT_DEBUG 0
#define _TIMERINTERRUPT_LOGLEVEL_ 0

#define USE_TIMER_1 false
#define USE_TIMER_2 false
#define USE_TIMER_3 true
#define USE_TIMER_4 false
#define USE_TIMER_5 false

#include "TimerInterrupt.h"
#include "ISR_Timer.h"


namespace interrupt
{
  // Wheel RPM timer
  static constexpr float kZeroRpm = 7500;                            // midpoint of min and max  // true 15500
  static constexpr float kMinRpm = 1000 - kZeroRpm;                  // actual min - kZero
  static constexpr float kMaxRpm = 14000 - kZeroRpm;                 // actual max - kZero  // true 30000
  static constexpr float kGlobalRate = 10000;                        // Hz of timer
  static constexpr float kRotPerFG = 1;                              // Ticks per
  static constexpr uint8_t kSecPerMin = 60;                          // seconds per minute
  volatile uint32_t prev_time[rw_status::kNumWheels] = {0, 0, 0, 0}; // 0.1 ms
  volatile float wheel_rpm[rw_status::kNumWheels] = {0, 0, 0, 0};    // index to motors
  volatile uint32_t global_time;                                     // 0.1 ms
  // interrupt functions to calculate the rpm of each motor

  // TODO: there is an issue where it seems that placing these in RwStatus.cpp can't be handled by platformio.
  // Look into that

  // global timer used for rpm calculation
  void TimerHandler()
  {
    global_time++;
  }

  // these must have "void FunctionName()" signature
  void ReadRpm0()
  {
    wheel_rpm[0] = (kSecPerMin * kRotPerFG * kGlobalRate / (float)(global_time - prev_time[0])) - kZeroRpm;
    prev_time[0] = global_time;
  }
  void ReadRpm1()
  {
    wheel_rpm[1] = (kSecPerMin * kRotPerFG * kGlobalRate / (float)(global_time - prev_time[1])) - kZeroRpm;
    prev_time[1] = global_time;
  }
  void ReadRpm2()
  {
    wheel_rpm[2] = (kSecPerMin * kRotPerFG * kGlobalRate / (float)(global_time - prev_time[2])) - kZeroRpm;
    prev_time[2] = global_time;
  }
  void ReadRpm3()
  {
    wheel_rpm[3] = (kSecPerMin * kRotPerFG * kGlobalRate / (float)(global_time - prev_time[3])) - kZeroRpm;
    prev_time[3] = global_time;
  }
} // namespace interrupt
#endif // RW_SRC_RWSTATUSCHECKER_HPP_
