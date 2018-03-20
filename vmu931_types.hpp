#ifndef VMU931_TYPES_HPP_8FPGUSNI
#define VMU931_TYPES_HPP_8FPGUSNI

#include <cstdint>
#include <string>

namespace vmu931
{

using Timestamp = uint32_t; /*< milliseconds since sensor start-up */

struct Data3D
{
    Timestamp timestamp = 0;
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

struct Accelerometers : public Data3D {};
struct Gyroscopes : public Data3D {};
struct Magnetometers : public Data3D {};
struct EulerAngles : public Data3D {};

struct Data4D
{
    Timestamp timestamp = 0;
    float w = 0.0f;
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

struct Quaternions : public Data4D {};

struct Heading
{
    Timestamp timestamp = 0;
    float heading = 0.0f; /*< [0.0, 360.0[ degree */
};

struct Status
{
    uint8_t sensors_status;
    uint8_t sensors_resolution;
    uint8_t low_output_rate_status;
    uint32_t data_currently_streaming;

    constexpr bool magnetometers() const { return sensors_status & 0x04; }
    constexpr bool gyroscopes() const { return sensors_status & 0x02; }
    constexpr bool accelerometers() const { return sensors_status & 0x01; }

    int resolution_gyroscopes() const;
    int resolution_accelerometers() const;
    int output_rate() const;

    std::string streaming() const;
};

} // namespace vmu931

#endif /* VMU931_TYPES_HPP_8FPGUSNI */

