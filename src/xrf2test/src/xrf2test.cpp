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
    logger.addVariable("this_is_a_int", data_to_be_logged.this_is_a_int);
    logger.addVariable("this_is_a_double", data_to_be_logged.this_is_a_double);
    logger.addVariable("this_is_a_float", data_to_be_logged.this_is_a_float);
    logger.addVariable("this_is_a_char", data_to_be_logged.this_is_a_char);
    logger.addVariable("this_is_a_bool", data_to_be_logged.this_is_a_bool);

    // Logging κινητήρων (ROS inputs)
    logger.addVariable("motor_left", data_to_be_logged.motor_left);
    logger.addVariable("motor_right", data_to_be_logged.motor_right);

    controller.SetFinishTime(0.0);  // Infinite run
}

xrf2test::~xrf2test() {}

int xrf2test::initialising()
{
    evl_printf("Hello from initialising\n");

    logger.initialise();    // Only once
    ico_io.init();          // FPGA (if used)

    // Άνοιγμα του Xenomai xbuffer (Ros → Xeno)
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
    logger.start();
    monitor.printf("Hello from run\n");

    // Ανάγνωση δεδομένων από ROS μέσω xbuffer
    xrf2_msgs::msg::Ros2Xeno ros_data;
    int read_size = read(xeno_fd, &ros_data, sizeof(ros_data));

    if (read_size > 0) {
        // Καταγραφή και αποθήκευση input
        u[0] = ros_data.left_motor;
        u[1] = ros_data.right_motor;

        data_to_be_logged.motor_left = ros_data.left_motor;
        data_to_be_logged.motor_right = ros_data.right_motor;
    } else {
        monitor.printf("No ROS data received this cycle\n");
    }

    // Στατική τροποποίηση dummy δεδομένων
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

    // Εκτέλεση ελέγχου
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
