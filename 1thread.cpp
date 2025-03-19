#include <iostream>
#include <fstream>
#include <pthread.h>
#include <evl/clock.h>
#include <evl/thread.h>
#include <evl/timer.h>
#include <evl/poll.h>
#include <cstring>
#include <errno.h>
#include <unistd.h>
#include <sched.h>

#define PERIOD_NS 1000000  // 1ms period
#define NUM_ITERATIONS 100
#define DEBUG 1  // Enable (1) or Disable (0) Debug Messages

std::ofstream csv_file("timing_data.csv"); // Open CSV file

void *periodic_task(void *arg) {
    std::string thread_name = "rt_thread";

    if (DEBUG) std::cout << "[DEBUG] Thread initialized.\n";

    // Attach thread to Xenomai EVL core
    int ret = evl_attach_self(thread_name.c_str());
    if (ret < 0) {
        std::cerr << "[ERROR] Thread failed to attach: " << strerror(-ret) << "\n";
        return NULL;
    }

    // Create periodic timer
    int timer_fd = evl_create_timer(EVL_CLOCK_MONOTONIC);
    if (timer_fd < 0) {
        std::cerr << "[ERROR] Failed to create timer: " << strerror(-timer_fd) << "\n";
        return NULL;
    }

    // Initialize timer to fire every 1ms
    struct itimerspec ts;
    ts.it_interval = {.tv_sec = 0, .tv_nsec = PERIOD_NS};
    ts.it_value = ts.it_interval;  // First expiration after 1ms

    ret = evl_set_timer(timer_fd, &ts, NULL);
    if (ret < 0) {
        std::cerr << "[ERROR] Failed to start timer: " << strerror(-ret) << "\n";
        return NULL;
    }

    struct timespec start_time, current_time;

    // Write CSV headers
    csv_file << "Iteration,Expected Time (ns),Actual Time (ns),Jitter (ns)\n";

    // Record the start time
    evl_read_clock(EVL_CLOCK_MONOTONIC, &start_time);

    // Main loop: poll timer and record timing info
    for (int i = 0; i < NUM_ITERATIONS; ++i) {
        // Poll the timer file descriptor
        struct evl_poll_event pe;
        ret = evl_poll(timer_fd, &pe, 1);
        if (ret < 0) {
            std::cerr << "[ERROR] Failed to poll timer: " << strerror(-ret) << "\n";
            break;
        }

        // Once the timer is ready, get the current time
        evl_read_clock(EVL_CLOCK_MONOTONIC, &current_time);

        // Calculate the expected and actual times
        long expected_time = (start_time.tv_sec * 1e9 + start_time.tv_nsec) + 
                             (i + 1) * PERIOD_NS;
        long actual_time = current_time.tv_sec * 1e9 + current_time.tv_nsec;
        long jitter = actual_time - expected_time;

        // Log to CSV
        csv_file << i + 1 << "," << expected_time << "," << actual_time << "," << jitter << "\n";

        if (DEBUG) {
            std::cout << "[DEBUG] Iteration " << i + 1
                      << " | Expected: " << expected_time
                      << " ns | Actual: " << actual_time
                      << " ns | Jitter: " << jitter << " ns\n";
        }
    }

    csv_file.close();
    std::cout << "[INFO] Timing data saved to timing_data.csv\n";
    return NULL;
}

int main() {
    pthread_t thread;

    if (pthread_create(&thread, NULL, periodic_task, NULL) != 0) {
        std::cerr << "[ERROR] Failed to create thread.\n";
        return 1;
    }

    pthread_join(thread, NULL);
    std::cout << "[INFO] Real-time thread completed execution.\n";
    return 0;
}
