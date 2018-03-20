#include "vmu931_sensor.hpp"
#include "vmu931_commands.hpp"
#include <algorithm>
#include <cassert>
#include <type_traits>
#include <boost/asio/write.hpp>
#include <boost/endian/conversion.hpp>

namespace vmu931
{

template<typename T>
T read(const uint8_t* ptr);

template<>
uint32_t read(const uint8_t* ptr)
{
    uint32_t v = *reinterpret_cast<const uint32_t*>(ptr);
    boost::endian::big_to_native_inplace(v);
    return v;
}

template<>
float read(const uint8_t* ptr)
{
    uint32_t tmp = read<uint32_t>(ptr);
    return *reinterpret_cast<float*>(&tmp);
}

template<>
uint8_t read(const uint8_t* ptr)
{
    return *ptr;
}

template<typename T>
T read_3d(const uint8_t* ptr)
{
    static_assert(std::is_base_of<Data3D, T>::value, "vmu931::Data3D derived type required");
    T data;
    data.timestamp = vmu931::read<uint32_t>(ptr);
    data.x = vmu931::read<float>(ptr + 4);
    data.y = vmu931::read<float>(ptr + 8);
    data.z = vmu931::read<float>(ptr + 12);
    return data;
}

template<typename T>
T read_4d(const uint8_t* ptr)
{
    static_assert(std::is_base_of<Data4D, T>::value, "vmu931::Data4D derived type required");
    T data;
    data.timestamp = vmu931::read<uint32_t>(ptr);
    data.w = vmu931::read<float>(ptr + 4);
    data.x = vmu931::read<float>(ptr + 8);
    data.y = vmu931::read<float>(ptr + 12);
    data.z = vmu931::read<float>(ptr + 16);
    return data;
}

Sensor::Sensor(boost::asio::serial_port&& port) :
    m_serial_port(std::move(port)), m_filled(0)
{
    read();
}

void Sensor::read()
{
    assert(m_buffer.size() >= m_filled);
    uint8_t* bufptr = m_buffer.data() + m_filled;
    std::size_t buflen = m_buffer.size() - m_filled;
    m_serial_port.async_read_some(boost::asio::buffer(bufptr, buflen),
        [this](const boost::system::error_code& ec, std::size_t bytes_transferred) {
            if (!ec) {
                m_filled += bytes_transferred;
                parse();
                read();
            }
        });
}

void Sensor::parse()
{
    const uint8_t* bufend = m_buffer.data() + m_filled;
    const uint8_t* msg = nullptr;
    for (const uint8_t* ptr = m_buffer.data(); ptr < bufend; ++ptr) {
        if (*ptr == 0x01 || *ptr == 0x02) {
            msg = ptr;
            break;
        }
    }

    if (msg) {
        const std::size_t len = bufend - msg;
        if (len < 3 || len < msg[1]) {
            trim(msg + 1);
        } else if (msg[0] == 0x01 && msg[len - 1] == 0x04) {
            parse_data(msg[2], &msg[3], &msg[len - 1]);
            trim(msg + len);
        } else if (msg[0] == 0x02 && msg[len - 1] == 0x03) {
            if (m_string) {
                std::string str;
                str.assign(&msg[3], &msg[len - 1]);
                m_string(str);
            }
            trim(msg + len);
        } else {
            trim(msg + 1);
        }
    } else {
        m_filled = 0;
    }
}

void Sensor::trim(const uint8_t* ptr)
{
    assert(ptr >= m_buffer.data());
    assert(ptr <= m_buffer.data() + m_buffer.size());
    const std::size_t len = m_buffer.data() + m_filled - ptr;
    std::memmove(m_buffer.data(), ptr, len);
    m_filled = len;
}

void Sensor::parse_data(char type, const uint8_t* begin, const uint8_t* end)
{
    const std::size_t len = end - begin;
    if (type == commands::Accelerometers && len == 16 && m_accel) {
        m_accel(read_3d<Accelerometers>(begin));
    } else if (type == commands::Gyroscopes && len == 16 && m_gyro) {
        m_gyro(read_3d<Gyroscopes>(begin));
    } else if (type == commands::Magnetometers && len == 16 && m_magneto) {
        m_magneto(read_3d<Magnetometers>(begin));
    } else if (type == commands::Quaternions && len == 20 && m_quat) {
        m_quat(read_4d<Quaternions>(begin));
    } else if (type == commands::EulerAngles && len == 16 && m_euler) {
        m_euler(read_3d<EulerAngles>(begin));
    } else if (type == commands::Heading && len == 8 && m_heading) {
        vmu931::Heading heading;
        heading.timestamp = vmu931::read<uint32_t>(begin);
        heading.heading = vmu931::read<float>(begin + 4);
        m_heading(heading);
    } else if (type == commands::Status && len == 7 && m_status) {
        vmu931::Status status;
        status.sensors_status = vmu931::read<uint8_t>(begin);
        status.sensors_resolution = vmu931::read<uint8_t>(begin + 1);
        status.low_output_rate_status = vmu931::read<uint8_t>(begin + 2);
        status.data_currently_streaming = vmu931::read<uint32_t>(begin + 3);
        m_status(status);
    }
}

bool Sensor::send_command(char command)
{
    std::string command_str("var");
    command_str.push_back(command);
    return boost::asio::write(m_serial_port, boost::asio::buffer(command_str)) == command_str.size();
}

void Sensor::register_sink(AccelerometersSink handler)
{
    m_accel = handler;
}

void Sensor::register_sink(GyroscopesSink handler)
{
    m_gyro = handler;
}

void Sensor::register_sink(MagnetometersSink handler)
{
    m_magneto = handler;
}

void Sensor::register_sink(EulerAnglesSink handler)
{
    m_euler = handler;
}

void Sensor::register_sink(QuaternionsSink handler)
{
    m_quat = handler;
}

void Sensor::register_sink(HeadingSink handler)
{
    m_heading = handler;
}

void Sensor::register_sink(StatusSink handler)
{
    m_status = handler;
}

void Sensor::register_sink(StringSink handler)
{
    m_string = handler;
}

} // namespace vmu931
