#include "vmu931/types.hpp"
#include "vmu931/commands.hpp"
#include <cassert>
#include <unordered_map>

namespace vmu931
{
namespace
{

static const std::unordered_map<char, uint32_t> stream_bits {
    {commands::Accelerometers, 0x01},
    {commands::Gyroscopes, 0x02},
    {commands::Quaternions, 0x04},
    {commands::Magnetometers, 0x08},
    {commands::EulerAngles, 0x10},
    {commands::Heading, 0x40}
};

} // namespace

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
    for (auto& stream_bit : stream_bits) {
        if (data_currently_streaming & stream_bit.second) {
            streams.push_back(stream_bit.first);
        }
    }
    return streams;
}

bool Status::is_streaming(char s) const
{
    auto found = stream_bits.find(s);
    if (found != stream_bits.end()) {
        return data_currently_streaming & found->second;
    } else {
        return false;
    }
}

} // namespace vmu931
