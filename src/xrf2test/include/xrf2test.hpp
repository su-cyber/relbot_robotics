#ifndef XRF2TEST_HPP
#define XRF2TEST_HPP

#include "XenoFrt20Sim.hpp"
#include "LoopController.h"

#pragma pack(push, 1)
struct ThisIsAStruct
{
    int this_is_a_int = 0;
    double this_is_a_double = 100.0;
    float this_is_a_float = 10.0;
    char this_is_a_char = 'R';
    bool this_is_a_bool = false;

    // Motor inputs received from ROS
    float motor_left = 0.0f;
    float motor_right = 0.0f;
};
#pragma pack(pop)

class xrf2test : public XenoFrt20Sim
{
public:
    xrf2test(uint write_decimator_freq, uint monitor_freq);
    ~xrf2test();

private:
    XenoFileHandler file;
    ThisIsAStruct data_to_be_logged;
    LoopController controller;

    double u[4]; // controller input (motor commands)
    double y[2]; // controller output

    int xeno_fd = -1; // xbuffer file descriptor

protected:
    int initialising() override;
    int initialised() override;
    int run() override;
    int stopping() override;
    int stopped() override;
    int pausing() override;
    int paused() override;
    int error() override;

    int current_error = 0;
};

#endif // XRF2TEST_HPP

