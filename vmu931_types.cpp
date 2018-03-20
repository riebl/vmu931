#include "vmu931_types.hpp"
#include "vmu931_commands.hpp"
#include <cassert>

namespace vmu931
{

int Status::resolution_gyroscopes() const
{
    int dps = -1;
    switch (sensors_resolution & 0xf0) {
        case 0x80:
            dps = 2000;
            break;
        case 0x40:
            dps = 1000;
            break;
        case 0x20:
            dps = 500;
            break;
        case 0x10:
            dps = 250;
            break;
        case 0x00:
            dps = 0;
            break;
        default:
            assert(false);
            break;
    }
    return dps;
}

int Status::resolution_accelerometers() const
{
    int g = -1;
    switch (sensors_resolution & 0x0f) {
        case 0x08:
            g = 16;
            break;
        case 0x04:
            g = 8;
            break;
        case 0x02:
            g = 4;
            break;
        case 0x01:
            g = 2;
            break;
        case 0x00:
            g = 0;
            break;
        default:
            assert(false);
            break;
    }
    return g;
}

int Status::output_rate() const
{
    return low_output_rate_status & 0x01 ? 200 : 1000;
}

std::string Status::streaming() const
{
    std::string streams;
    if (data_currently_streaming & 0x01) {
        streams.push_back(commands::Accelerometers);
    }
    if (data_currently_streaming & 0x02) {
        streams.push_back(commands::Gyroscopes);
    }
    if (data_currently_streaming & 0x04) {
        streams.push_back(commands::Quaternions);
    }
    if (data_currently_streaming & 0x08) {
        streams.push_back(commands::Magnetometers);
    }
    if (data_currently_streaming & 0x10) {
        streams.push_back(commands::EulerAngles);
    }
    if (data_currently_streaming & 0x40) {
        streams.push_back(commands::Heading);
    }
    return streams;
}

} // namespace vmu931
