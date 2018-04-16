#include <iostream>
#include <boost/asio.hpp>
#include "vmu931/commands.hpp"
#include "vmu931/sensor.hpp"
#include "vmu931/types.hpp"

int main()
{
    boost::asio::io_service io_service;
    boost::asio::serial_port serial_port(io_service, "/dev/ttyACM0");

    vmu931::Sensor sensor(std::move(serial_port));
    sensor.register_sink([](vmu931::Accelerometers accel) {
        std::cout << "accel: x=" << accel.x << " y=" << accel.y << " z=" << accel.z <<"\n";
    });
    sensor.register_sink([](vmu931::Gyroscopes gyro) {
        std::cout << "gyro: x=" << gyro.x << " y=" << gyro.y << " z=" << gyro.z <<"\n";
    });
    sensor.register_sink([](vmu931::Magnetometers magneto) {
        std::cout << "magneto: x=" << magneto.x << " y=" << magneto.y << " z=" << magneto.z <<"\n";
    });
    sensor.register_sink([](vmu931::EulerAngles euler) {
        std::cout << "euler: x=" << euler.x << " y=" << euler.y << " z=" << euler.z <<"\n";
    });
    sensor.register_sink([](vmu931::Quaternions quat) {
        std::cout << "quat: w=" << quat.w << " x=" << quat.x << " y=" << quat.y << " z=" << quat.z <<"\n";
    });
    sensor.register_sink([](vmu931::Heading h) {
        std::cout << "heading: " << h.heading << "\n";
    });
    sensor.register_sink([](std::string s) {
        std::cout << "message: " << s << "\n";
    });
    sensor.register_sink([](vmu931::Status status) {
        std::cout << "VMU931 status:\n"
            << " - streams: " << status.streaming() << "\n"
            << " - gyroscopes: " << status.resolution_gyroscopes() << " dps\n"
            << " - accelerometers: " << status.resolution_accelerometers() << " g\n"
            << " - output rate: " << status.output_rate() << " Hz\n";
    });

    io_service.post([&sensor]() {
            sensor.set_streams({vmu931::commands::Heading, vmu931::commands::EulerAngles});
    });

    std::cout << "Start reading VMU931 sensor stream...\n";
    io_service.run();

    return 0;
}
