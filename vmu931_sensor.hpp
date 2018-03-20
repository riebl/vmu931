#ifndef VMU931_SENSOR_HPP_LZK2XFWW
#define VMU931_SENSOR_HPP_LZK2XFWW

#include <array>
#include <functional>
#include <string>
#include <boost/asio/serial_port.hpp>
#include "vmu931_types.hpp"

namespace vmu931
{

class Sensor
{
public:
    using AccelerometersSink = std::function<void(vmu931::Accelerometers)>;
    using GyroscopesSink = std::function<void(vmu931::Gyroscopes)>;
    using MagnetometersSink = std::function<void(vmu931::Magnetometers)>;
    using EulerAnglesSink = std::function<void(vmu931::EulerAngles)>;
    using QuaternionsSink = std::function<void(vmu931::Quaternions)>;
    using HeadingSink = std::function<void(vmu931::Heading)>;
    using StatusSink = std::function<void(vmu931::Status)>;
    using StringSink = std::function<void(std::string)>;

    Sensor(boost::asio::serial_port&&);
    bool send_command(char);
    void register_sink(AccelerometersSink);
    void register_sink(GyroscopesSink);
    void register_sink(MagnetometersSink);
    void register_sink(EulerAnglesSink);
    void register_sink(QuaternionsSink);
    void register_sink(HeadingSink);
    void register_sink(StatusSink);
    void register_sink(StringSink);

private:
    void read();
    void parse();
    void trim(const uint8_t*);
    void parse_data(char type, const uint8_t* begin, const uint8_t* end);

    boost::asio::serial_port m_serial_port;
    std::array<uint8_t, 260> m_buffer;
    std::size_t m_filled;

    AccelerometersSink m_accel;
    GyroscopesSink m_gyro;
    MagnetometersSink m_magneto;
    EulerAnglesSink m_euler;
    QuaternionsSink m_quat;
    HeadingSink m_heading;
    StatusSink m_status;
    StringSink m_string;
};

} // namespace vmu931

#endif /* VMU931_SENSOR_HPP_LZK2XFWW */
