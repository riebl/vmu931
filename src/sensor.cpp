#include "vmu931/sensor.hpp"
#include "vmu931/commands.hpp"
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
    m_serial_port(std::move(port)), m_timer(m_serial_port.get_io_service()), m_filled(0)
{
    read();
}

void Sensor::read()
{
    assert(m_buffer.size() >= m_filled);
    uint8_t* bufptr = m_buffer.data() + m_filled;
    std::size_t buflen = m_buffer.size() - m_filled;

    m_timer.expires_from_now(boost::posix_time::milliseconds(15));
    m_timer.async_wait([this](const boost::system::error_code& ec) {
        if (!ec && !m_command.empty()) {
            m_serial_port.cancel();
        }
    });

    assert(buflen > 0);
    m_serial_port.async_read_some(boost::asio::buffer(bufptr, buflen),
        [this](const boost::system::error_code& ec, std::size_t bytes_transferred) {
            m_filled += bytes_transferred;
            if (!ec) {
                while (parse()) {}
                if (m_command.empty()) {
                    read();
                } else {
                    m_timer.cancel();
                    write();
                }
            } else if (ec == boost::asio::error::operation_aborted) {
                write();
            }
        });
}

void Sensor::write()
{
    assert(!m_command.empty());
    boost::asio::async_write(m_serial_port, boost::asio::buffer(m_command),
        [this](const boost::system::error_code& ec, std::size_t bytes_transferred) {
            if (!ec) {
                assert(bytes_transferred == m_command.size());
                m_command.clear();
                read();
            } else if (ec == boost::asio::error::operation_aborted) {
                m_command.erase(0, bytes_transferred);
                m_command.empty() ? read() : write();
            }
        });
}

bool Sensor::parse()
{
    bool parsed = false;
    const uint8_t* bufend = m_buffer.data() + m_filled;
    const uint8_t* msg = nullptr;
    for (const uint8_t* ptr = m_buffer.data(); ptr < bufend; ++ptr) {
        if (*ptr == 0x01 || *ptr == 0x02) {
            msg = ptr;
            break;
        }
    }

    if (msg) {
        const std::size_t buflen = bufend - msg;
        if (buflen < 2 || buflen < msg[1]) {
            trim(msg);
        } else {
            const auto msglen = msg[1];
            if (msg[0] == 0x01 && msg[msglen - 1] == 0x04) {
                parse_data(msg[2], &msg[3], &msg[msglen - 1]);
                trim(msg + msglen);
                parsed = true;
            } else if (msg[0] == 0x02 && msg[msglen - 1] == 0x03) {
                if (m_string) {
                    std::string str;
                    str.assign(&msg[3], &msg[msglen - 1]);
                    m_string(str);
                }
                trim(msg + msglen);
                parsed = true;
            } else {
                trim(msg + 1);
            }
        }
    } else {
        m_filled = 0;
    }

    return parsed;
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
    } else if (type == commands::Status && len == 7) {
        vmu931::Status status;
        status.sensors_status = vmu931::read<uint8_t>(begin);
        status.sensors_resolution = vmu931::read<uint8_t>(begin + 1);
        status.low_output_rate_status = vmu931::read<uint8_t>(begin + 2);
        status.data_currently_streaming = vmu931::read<uint32_t>(begin + 3);
        if (m_pending_streams) {
            toggle_streams(status);
        } else if (m_status) {
            m_status(status);
        }
    }
}

void Sensor::send_command(char command)
{
    m_command.append("var");
    m_command.push_back(command);
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

void Sensor::set_streams(const std::unordered_set<char>& streams)
{
    m_pending_streams = streams;
    send_command(commands::Status);
}

void Sensor::toggle_streams(const Status& status)
{
    if (m_pending_streams) {
        for (char possible_stream : commands::Data)
        {
            const bool want_stream = m_pending_streams->count(possible_stream);
            const bool has_stream = status.is_streaming(possible_stream);
            if (want_stream != has_stream) {
                send_command(possible_stream);
            }
        }
        m_pending_streams = boost::none;
    }
}

} // namespace vmu931
