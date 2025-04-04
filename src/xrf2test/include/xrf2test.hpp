#ifndef XRF2TEST_HPP
#define XRF2TEST_HPP

#include "XenoFrt20Sim.hpp"
#include "LoopController.h"

#pragma pack(1)
struct ThisIsAStruct
{
    int this_is_a_int = 0;
    double this_is_a_double = 100.0;
    float this_is_a_float = 10.0;
    char this_is_a_char = 'R';
    bool this_is_a_bool = false;
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
