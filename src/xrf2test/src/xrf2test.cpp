#include "xrf2test.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <cstring>
#include <iostream>
#include "xrf2_msgs/msg/ros2_xeno.hpp"

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

    // Logging ROS input commands
    logger.addVariable("motor_left", float_);
    logger.addVariable("motor_right", float_);

    controller.SetFinishTime(0.0);  // Infinite run
}

xrf2test::~xrf2test() {}

int xrf2test::initialising()
{
    evl_printf("Hello from initialising\n");

    logger.initialise();    // Only once
    ico_io.init();          // FPGA (if used)

    // Open Xenomai xbuffer (ROS → Xeno)
    xeno_fd = open("/dev/evl/xbuf/Ros-Xeno", O_RDWR);
    if (xeno_fd < 0) {
        evl_printf("Failed to open /dev/evl/xbuf/Ros-Xeno: %s\n", strerror(errno));
    }

    return 1;
}

int xrf2test::initialised()
{
    evl_printf("Hello from initialised\n");
    return 0;
}

int xrf2test::run()
{

    monitor.printf("Received left_motor: %.4f\n", ros_msg.left_motor);
    monitor.printf("Received right_motor: %.4f\n", ros_msg.right_motor);

    // Prepare input for controller
    u[0] = 0;
    u[1] = 0;
    u[2] = ros_msg.left_motor;  // Typically enable signals or fixed inputs
    u[3] = ros_msg.right_motor;

    // Run the 20-sim model
    controller.Calculate(u, y);

    // Apply output to actuators
    actuate_data.pwm1 = static_cast<int16_t>(-y[1]);  // Right wheel
    actuate_data.pwm2 = static_cast<int16_t>(y[0]); // Left wheel (inverted)


    // Dummy data updates for logger
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

    // Exit FSM if controller finished (never in infinite mode)
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
