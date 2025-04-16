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

    // ew variables for 3.2 logging
    double pos_left_rad = 0.0;
    double pos_right_rad = 0.0;
    int delta_left = 0;
    int delta_right = 0;
    int pwm_left = 0;
    int pwm_right = 0;
    double accumulated_pos_left = 0.0;
    double accumulated_pos_right = 0.0;
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

    // Variables to be logged
    double pos_left_rad = 0.0;
    double pos_right_rad = 0.0;
    int delta_left = 0;
    int delta_right = 0;

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
