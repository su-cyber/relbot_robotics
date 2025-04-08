#ifndef TEMPLATE20SIM_HPP
#define TEMPLATE20SIM_HPP

#include "XenoFrt20Sim.hpp"
#include "LoopController.h"

#pragma pack (1)    // https://carlosvin.github.io/langs/en/posts/cpp-pragma-pack/#_performance_test
struct ThisIsAStruct
{
    int this_is_a_int = 0;
    double this_is_a_double = 100.0;
    float this_is_a_float = 10.0;
    char this_is_a_char = 'R';
    bool this_is_a_bool = false;

    // 👇 Προσθήκη: logging των εντολών κινητήρων
    double motor_left = 0.0;
    double motor_right = 0.0;
};
#pragma pack(0)

class xrf2test : public XenoFrt20Sim
{
public:
    xrf2test(uint write_decimator_freq, uint monitor_freq);
    ~xrf2test();
private:
    XenoFileHandler file;
    struct ThisIsAStruct data_to_be_logged;
    LoopController controller;

    double u[2];
    double y[2];
    int xeno_fd;

protected:
    // State machine methods
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

#endif // TEMPLATE20SIM_HPP
