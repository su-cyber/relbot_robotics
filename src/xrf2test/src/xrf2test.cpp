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
    u[0] = 0.0;
    u[1] = 0.0;
    y[0] = 0.0;
    y[1] = 0.0;
}

xrf2test::~xrf2test() {}

int xrf2test::initialising()
{
    std::cout << "xrf2test: initialising()" << std::endl;
    return 0;
}

int xrf2test::initialised()
{
    return 0;
}

int xrf2test::run()
{
    return 0;
}

int xrf2test::stopping()
{
    return 0;
}

int xrf2test::stopped()
{
    return 0;
}

int xrf2test::pausing()
{
    return 0;
}

int xrf2test::paused()
{
    return 0;
}

int xrf2test::error()
{
    return current_error;
}
