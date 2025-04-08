#include "xrf2test.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <cstring>
#include <iostream>
#include "xrf2_msgs/msg/ros2_xeno.hpp"

xrf2test::xrf2test(uint write_decimator_freq, uint monitor_freq)
: XenoFrt20Sim(write_decimator_freq, monitor_freq)
{
    printf("%s: Constructing xrf2test\n", __FUNCTION__);
    controller.SetFinishTime(0.0);  // Infinite run
}

xrf2test::~xrf2test() {}

int xrf2test::initialising()
{
    evl_printf("Hello from initialising\n");

    ico_io.init();  // FPGA (if used)

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
    monitor.printf("Hello from run\n");

    // Ανάγνωση δεδομένων από ROS μέσω xbuffer
    xrf2_msgs::msg::Ros2Xeno ros_data;
    int read_size = read(xeno_fd, &ros_data, sizeof(ros_data));

    if (read_size > 0) {
        u[0] = ros_data.left_motor;
        u[1] = ros_data.right_motor;
    } else {
        monitor.printf("No ROS data received this cycle\n");
    }

    // Εκτέλεση ελέγχου
    controller.Calculate(u, y);

    if (controller.IsFinished())
        return 1;

    return 0;
}

int xrf2test::stopping()
{
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
