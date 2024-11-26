#ifndef RW_SRC_MAIN_HPP_
#define RW_SRC_MAIN_HPP_

#include "Arduino.h"
#include <Adafruit_BNO08x.h>
#include <imumaths.hpp>
#include "SD.h"

/* Physical Info */
namespace physical
{
    // physical constants
    static constexpr uint8_t kNumWheels = 4;

    // pins
    const uint8_t kPwmPins[kNumWheels] = {10, 11, 12, 13};
    const uint8_t kDirectionPins[kNumWheels] = {44, 40, 36, 32};
    const uint8_t kFgPins[kNumWheels] = {18, 19, 2, 3}; // rpm reading pins

    // signals
    uint8_t pwm_signal[kNumWheels] = {0, 0, 0, 0};       // 0-255
    uint8_t direction_signal[kNumWheels] = {1, 1, 1, 1}; // 1 clockwise, 0 ccw

    // imu
    Adafruit_BNO08x bno(-1); // we use I2C for the IMU, so this is unnecessary

    // SD reader
    static constexpr uint8_t kChipSelect = 53; // mega specific number
    static constexpr uint8_t kSdPin = 10;      // pin on the SD reader
    File file;                                 // file opened on the SD card
} // namespace physical

/* Serial Info */
namespace serial_stuff
{
    static constexpr int kSerialRate = 115200;
    static constexpr int cycles_per_print = 10;

    static constexpr char start_byte = '<';
    static constexpr char stop_byte = '>';

    static constexpr char MCU_init_phrase[] = "<MCU_init>\n";

    static constexpr uint8_t msg_key[] = {
        0, // "stop"

        10, //  "set_quaternion"
        11, //  "get_quaternion"

        20, //  "set_rpm"
        21, //  "get_rpm"

        30, //  "get_pwm"

        254, //  "reserved"
        255, //  "reserved"
    };

    typedef union
    {
        float number;
        uint8_t bytes[4];
    } FLOATUNION_t;

    static constexpr int max_serial_input = 20;

    uint8_t current_command;
    byte current_argument[max_serial_input];
    byte unparsed_from_serial = false;

    static void write_quaternion_to_serial(const uint8_t key, imu::Quaternion to_write)
    {
        serial_stuff::FLOATUNION_t to_write_floats[4] = {{to_write.w()}, {to_write.x()}, {to_write.y()}, {to_write.z()}};

        Serial.write(serial_stuff::start_byte);
        Serial.write(static_cast<byte>(key)); //  "get_quaternion"
        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                Serial.write(to_write_floats[i].bytes[j]);
            }
        }
        Serial.write(serial_stuff::stop_byte);
    }
    static void write_4_floats_to_serial(const uint8_t key, volatile float to_write[4])
    {
        serial_stuff::FLOATUNION_t to_write_floats[4];
        memcpy(to_write, to_write_floats, 4 * sizeof(float));

        Serial.write(serial_stuff::start_byte);
        Serial.write(static_cast<byte>(key)); //  "get_quaternion"
        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                Serial.write(to_write_floats[i].bytes[j]);
            }
        }
        Serial.write(serial_stuff::stop_byte);
    }

    /*
        static byte serial_data[20];
        static bool data_to_send = false;
        static void send_bytes_to_serial()
        {
            if (serial_stuff::data_to_send)
            {
                Serial.write(start_byte);
                for (byte single_byte : serial_stuff::serial_data)
                {
                    Serial.write(single_byte);
                    if (single_byte == stop_byte)
                    {
                        serial_stuff::data_to_send = false;
                        return;
                    }
                }
            }
        }
    */
} // namespace serial_stuff

/* Timing */
namespace timer
{
    uint8_t init_time;             // time in ms of finish setup
    uint32_t loop_start_time;      // time in ms of this loop
    uint32_t loop_prev_start_time; // time in ms of prev loop
    uint32_t loop_dt;              // delta between current and prev loop
} // namespace timer

/* Utility functions*/
namespace util
{
    // returns 1 if x > 0, 0 if x = 0, -1 if x < 0
    static inline int8_t sign(double x) { return (x < 0) ? -1 : ((x > 0) ? 1 : 0); }
} // namespace util

/* configurable parameters for testing */
namespace test_parameters
{
    static unsigned timeout_sec = 300;
    static unsigned int test_delay = 15;

    static constexpr float torque_PD_params[2] = {1.f, 0.f};       // quaternion torque controller PD params
    static constexpr float wheel_speed_PD_params[2] = {1e-3, 0.f}; // wheel speed controller PD params

} // namespace test_parameters

#endif // RW_SRC_MAIN_HPP_
