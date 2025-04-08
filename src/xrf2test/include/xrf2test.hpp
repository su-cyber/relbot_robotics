#ifndef XRF2TEST_HPP
#define XRF2TEST_HPP

#include "XenoFrt20Sim.hpp"
#include "LoopController.h"

class xrf2test : public XenoFrt20Sim
{
public:
    xrf2test(uint write_decimator_freq, uint monitor_freq);
    ~xrf2test();

private:
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

#endif // XRF2TEST_HPP
