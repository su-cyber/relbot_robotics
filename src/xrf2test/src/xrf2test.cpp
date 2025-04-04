#include "xrf2test.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <cstring>
#include <iostream>
#include "xrf2_msgs/msg/ros2_xeno.hpp"

// Struct to match ROS 2 Ros2Xeno message layout
#pragma pack(push, 1)
struct Ros2XenoRT {
    int32_t sec;
    uint32_t nanosec;
    char frame_id[64];  // Max 63 chars + null terminator
    double left_motor;
    double right_motor;
};
#pragma pack(pop)

xrf2test::xrf2test(uint write_decimator_freq, uint monitor_freq)
: XenoFrt20Sim(write_decimator_freq, monitor_freq, file, &data_to_be_logged),
  file(1, "./xrf2_logging/XRF2TEST", "bin"),
  controller()
{
    printf("%s: Constructing xrf2test\n", __FUNCTION__);

    // Add variables to logger before logging starts
    logger.addVariable("this_is_a_int", integer);
    logger.addVariable("this_is_a_double", double_);
    logger.addVariable("this_is_a_float", float_);
    logger.addVariable("this_is_a_char", character);
    logger.addVariable("this_is_a_bool", boolean);

    controller.SetFinishTime(0.0);  // Infinite run
}

xrf2test::~xrf2test() {}

int xrf2test::initialising()
{
    evl_printf("Hello from initialising\n");

    logger.initialise();    // Only once
    ico_io.init();          // FPGA (if used)

    return 1;
}

int xrf2test::initialised()
{
    evl_printf("Hello from initialised\n");
    return 0;
}

int xrf2test::run()
{
    logger.start();
    monitor.printf("Hello from run\n");

    // ✅ NEW: Read motor command from ROS bridge via EVL
    Ros2XenoRT msg;
    ssize_t ret = read(data_fd, &msg, sizeof(msg));

    if (ret > 0) {
        u[0] = msg.left_motor;
        u[1] = msg.right_motor;

        monitor.printf("Received: L=%.2f, R=%.2f at %d.%09u (frame_id=%s)\n",
                       msg.left_motor, msg.right_motor, msg.sec, msg.nanosec, msg.frame_id);
    }

    // Log data and simulate sensor state change
    data_to_be_logged.this_is_a_bool = !data_to_be_logged.this_is_a_bool;
    data_to_be_logged.this_is_a_int++;

    if (data_to_be_logged.this_is_a_char == 'R')
        data_to_be_logged.this_is_a_char = 'A';
    else if (data_to_be_logged.this_is_a_char == 'A')
        data_to_be_logged.this_is_a_char = 'M';
    else
        data_to_be_logged.this_is_a_char = 'R';

    data_to_be_logged.this_is_a_float /= 2;
    data_to_be_logged.this_is_a_double /= 4;

    controller.Calculate(u, y);

    if (controller.IsFinished())
        return 1;

    return 0;
}

int xrf2test::stopping()
{
    logger.stop();
    evl_printf("Hello from stopping\n");
    return 1;
}

int xrf2test::stopped()
{
    monitor.printf("Hello from stopped\n");
    return 0;
}

int xrf2test::pausing()
{
    evl_printf("Hello from pausing\n");
    return 1;
}

int xrf2test::paused()
{
    monitor.printf("Hello from paused\n");
    return 0;
}

int xrf2test::error()
{
    monitor.printf("Hello from error\n");
    return 0;
}
