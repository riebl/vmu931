#ifndef VMU931_COMMANDS_HPP_WIUGMQHB
#define VMU931_COMMANDS_HPP_WIUGMQHB

#include <unordered_set>

namespace vmu931
{
namespace commands
{

// data commands
constexpr char Accelerometers = 'a';
constexpr char Gyroscopes = 'g';
constexpr char Magnetometers ='c';
constexpr char Quaternions = 'q';
constexpr char EulerAngles = 'e';
constexpr char Heading = 'h';

static const std::unordered_set<char> Data {
    Accelerometers, Gyroscopes, Magnetometers, Quaternions, EulerAngles, Heading
};

// misc commands
constexpr char SelfTest = 't';
constexpr char Calibration = 'l';
constexpr char Status = 's';

// resolution commands
constexpr char Gyroscopes_250dps = '0';
constexpr char Gyroscopes_500dps = '1';
constexpr char Gyroscopes_1000dps = '2';
constexpr char Gyroscopes_2000dps = '3';
constexpr char Accelerometers_2g = '4';
constexpr char Accelerometers_4g = '5';
constexpr char Accelerometers_8g = '6';
constexpr char Accelerometers_16g = '7';

} // namespace commands
} // namespace vmu931

#endif /* VMU931_COMMANDS_HPP_WIUGMQHB */
