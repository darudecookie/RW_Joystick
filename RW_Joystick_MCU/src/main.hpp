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

    static constexpr char start_byte = static_cast<char>(254);
    static constexpr char stop_byte = static_cast<char>(255);

    static constexpr char MCU_init_phrase[] = "<MCU_init>\n";

    static constexpr uint8_t msg_key[] = {
        0,

        10,
        11,

        20,

        254,
        255,
    };

    static constexpr char msg_val[][16] = {
        "stop",

        "set_quaternion",
        "get_quaternion",

        "get_rpm",

        "reserved",
        "reserved",
    };

    static float decode_1_float(byte byte_array[3])
    {
        float parsed_float = 0;

        if (static_cast<uint8_t>(byte_array[0]) >= 101)
        {
            parsed_float = -(static_cast<uint8_t>(byte_array[0]) - 101);
            parsed_float -= (static_cast<float>(byte_array[1]) - 101) / 100;
            parsed_float -= (static_cast<float>(byte_array[2]) - 101) / (100 * 100);
        }
        else
        {
            parsed_float = static_cast<int>(byte_array[0]);
            parsed_float += static_cast<float>(byte_array[1]) / 100;
            parsed_float += static_cast<float>(byte_array[2]) / (100 * 100);
        }
        return parsed_float;
    }
    static void decode_4_floats(float target_array[4], byte input_bytes[12])
    {
        for (uint8_t i = 0; i < 4; i++)
        {
            byte byte_array[3] = {input_bytes[i * 3], input_bytes[i * 3 + 1], input_bytes[i * 3 + 2]};

            target_array[i] = decode_1_float(byte_array);
        }
    }

    static void encode_1_float(float input_float, byte modified_bytes[3])
    {
        uint8_t float_sections[3] = {
            (uint8_t)(input_float),
            (uint8_t)(input_float * 100) - (uint8_t)(input_float) * 100,
            (uint8_t)(input_float * 100 * 100) - (uint8_t)(input_float * 100) * 100,
        };

        for (byte i = 0; i < 3; i++)
        {
            if (float_sections[i] < 0)
            {
                modified_bytes[i] = abs(float_sections[i]) + 101;
            }
        }
    }
    static void encode_4_floats(float input_float_aray[4], byte modified_bytes[12])
    {
        for (byte i = 0; i < 12; i += 3)
        {
            byte input_bytes[3] = {
                modified_bytes[i], modified_bytes[i + 1], modified_bytes[i + 2]};
            encode_1_float(input_float_aray[i / 3], input_bytes);
        }
    }

    static void send_bytes_to_serial()
    {
        if (serial::data_to_send)
        {
            Serial.write(start_byte);
            for (byte single_byte : serial_data)
            {
                Serial.write(single_byte);
                if (single_byte == stop_byte)
                {
                    serial::data_to_send = false;
                    return;
                }
            }
        }
    }

    static byte serial_data[20];
    static bool data_to_send = false;

}

/* Timing */
namespace timer
{
    uint8_t init_time;             // time in ms of finish setup
    uint32_t loop_start_time;      // time in ms of this loop
    uint32_t loop_prev_start_time; // time in ms of prev loop
    uint32_t loop_dt;              // delta between current and prev loop
} // global time

/* Utility functions*/
namespace util
{
    // returns 1 if x > 0, 0 if x = 0, -1 if x < 0
    static inline int8_t sign(double x) { return (x < 0) ? -1 : ((x > 0) ? 1 : 0); }
}

namespace test_parameters
{
    static int timeout_sec = 300;
    static int test_delay = 15;

    static constexpr float torque_PD_params[2] = {1.f, 0.f};       // quaternion torque controller PD params
    static constexpr float wheel_speed_PD_params[2] = {1e-3, 0.f}; // wheel speed controller PD params

}
/* configurable parameters for testing */

#endif // RW_SRC_MAIN_HPP_
