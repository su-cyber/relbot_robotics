#include "xrf2test.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <cstring>
#include <iostream>
#include "xrf2_msgs/msg/ros2_xeno.hpp"
#include <algorithm>  // for std::clamp

// ---------------------------------------------
// Encoder delta with wraparound correction (14-bit)
// ---------------------------------------------
int compute_encoder_delta(int current, int previous)
{
    int delta = current - previous;

    if (delta > 8192)       // wrapped backwards
        delta -= 16384;
    else if (delta < -8192) // wrapped forwards
        delta += 16384;

    return delta;
}

xrf2test::xrf2test(uint write_decimator_freq, uint monitor_freq)
: XenoFrt20Sim(write_decimator_freq, monitor_freq, file, &data_to_be_logged),
  file(1, "./xrf2_logging/XRF2TEST", "bin"),
  controller()
{
    printf("%s: Constructing xrf2test\n", __FUNCTION__);

    logger.addVariable("this_is_a_int", integer);
    logger.addVariable("this_is_a_double", double_);
    logger.addVariable("this_is_a_float", float_);
    logger.addVariable("this_is_a_char", character);
    logger.addVariable("this_is_a_bool", boolean);
    logger.addVariable("motor_left", float_);
    logger.addVariable("motor_right", float_);

    controller.SetFinishTime(0.0);  // Infinite run

    // Logging actual runtime variable
    logger.addVariable("pos_left_rad", pos_left_rad);
    logger.addVariable("pos_right_rad", pos_right_rad);
    logger.addVariable("delta_left", delta_left);
    logger.addVariable("delta_right", delta_right);
    logger.addVariable("pwm_left", actuate_data.pwm2);
    logger.addVariable("pwm_right", actuate_data.pwm1);
}


xrf2test::~xrf2test() {}

int xrf2test::initialising()
{
    evl_printf("Hello from initialising\n");

    logger.initialise();
    ico_io.init();

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
    const double COUNTS_PER_WHEEL_REV = 1024.0 * 15.58;  // = 15939.2
    const double RAD_PER_COUNT = 2.0 * 3.14159265358979323846 / COUNTS_PER_WHEEL_REV;

    // Read current encoder values
    int enc_right = sample_data.channel1;
    int enc_left = sample_data.channel2;

    // Store previous encoder values between runs
    static int last_enc_right = 0;
    static int last_enc_left = 0;

    // Compute delta (change) in encoder counts, accounting for wraparound
    int delta_right = compute_encoder_delta(enc_right, last_enc_right);
    int delta_left  = compute_encoder_delta(enc_left, last_enc_left);

    // Save current encoders for next cycle
    last_enc_right = enc_right;
    last_enc_left  = enc_left;

    // Convert encoder counts to wheel position in radians
    double pos_right_rad = enc_right * RAD_PER_COUNT;
    double pos_left_rad = enc_left * RAD_PER_COUNT;

    // Optional: print deltas
    monitor.printf("Encoder ΔRight: %d | ΔLeft: %d\n", delta_right, delta_left);
    monitor.printf("Received left_motor: %.4f | right_motor: %.4f\n", ros_msg.left_motor, ros_msg.right_motor);
    monitor.printf("Encoder P1 (Right): %d | Encoder P2 (Left): %d\n", enc_right, enc_left);
    monitor.printf("Wheel pos [rad] to L: %.3f  R: %.3f\n", pos_left_rad, pos_right_rad);

    // Prepare input for controller
    u[0] = pos_left_rad;
    u[1] = pos_right_rad;
    u[2] = ros_msg.left_motor;
    u[3] = ros_msg.right_motor;

    controller.Calculate(u, y);

    // Convert control output to PWM
    const double SCALE = 2047.0 / 900.0;
    actuate_data.pwm1 = static_cast<int16_t>(std::clamp(-y[1] * SCALE, -2047.0, 2047.0));
    actuate_data.pwm2 = static_cast<int16_t>(std::clamp(y[0] * SCALE, -2047.0, 2047.0));

    // Dummy logger values
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
        return controller.IsFinished() ? 1 : 0;
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
