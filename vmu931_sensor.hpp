#ifndef VMU931_SENSOR_HPP_LZK2XFWW
#define VMU931_SENSOR_HPP_LZK2XFWW

#include <array>
#include <functional>
#include <string>
#include <unordered_set>
#include <boost/asio/deadline_timer.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/optional/optional.hpp>
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
    void send_command(char);
    void register_sink(AccelerometersSink);
    void register_sink(GyroscopesSink);
    void register_sink(MagnetometersSink);
    void register_sink(EulerAnglesSink);
    void register_sink(QuaternionsSink);
    void register_sink(HeadingSink);
    void register_sink(StatusSink);
    void register_sink(StringSink);
    void set_streams(const std::unordered_set<char>&);

private:
    void read();
    void write();
    bool parse();
    void trim(const uint8_t*);
    void parse_data(char type, const uint8_t* begin, const uint8_t* end);
    void toggle_streams(const Status&);

    boost::asio::serial_port m_serial_port;
    boost::asio::deadline_timer m_timer;
    std::array<uint8_t, 256> m_buffer;
    std::size_t m_filled;
    std::string m_command;
    boost::optional<std::unordered_set<char>> m_pending_streams;

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
