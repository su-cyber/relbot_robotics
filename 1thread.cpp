#include <iostream>
#include <fstream>
#include <pthread.h>
#include <evl/clock.h>
#include <evl/thread.h>
#include <evl/timer.h>
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

    struct evl_sched_attrs attrs;
    memset(&attrs, 0, sizeof(attrs));
    attrs.sched_policy = SCHED_FIFO;
    attrs.sched_priority = 10;

    int ret = evl_attach_thread(EVL_CLONE_PUBLIC, thread_name.c_str(), &attrs);
    if (ret < 0) {
        std::cerr << "[ERROR] Thread failed to attach: " << strerror(-ret) << "\n";
        return NULL;
    }

    // Set CPU affinity to Core 1 (Xenomai reserved)
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(1, &cpuset);
    ret = sched_setaffinity(0, sizeof(cpu_set_t), &cpuset);
    if (ret != 0) {
        std::cerr << "[ERROR] Failed to set CPU affinity.\n";
        return NULL;
    }

    // Create periodic timer
    int timer_fd = evl_new_timer(EVL_CLOCK_MONOTONIC, NULL);
    if (timer_fd < 0) {
        std::cerr << "[ERROR] Failed to create timer: " << strerror(-timer_fd) << "\n";
        return NULL;
    }

    struct itimerspec ts;
    ts.it_value.tv_sec = 0;
    ts.it_value.tv_nsec = PERIOD_NS;
    ts.it_interval.tv_sec = 0;
    ts.it_interval.tv_nsec = PERIOD_NS;

    ret = evl_set_timer(timer_fd, &ts);
    if (ret < 0) {
        std::cerr << "[ERROR] Failed to start timer: " << strerror(-ret) << "\n";
        return NULL;
    }

    struct timespec start_time, current_time;
    
    // Write CSV headers
    csv_file << "Iteration,Expected Time (ns),Actual Time (ns),Jitter (ns)\n";
    
    evl_read_clock(EVL_CLOCK_MONOTONIC, &start_time);

    for (int i = 0; i < NUM_ITERATIONS; ++i) {
        evl_read_clock(EVL_CLOCK_MONOTONIC, &current_time);
        
        long expected_time = start_time.tv_sec * 1e9 + start_time.tv_nsec + i * PERIOD_NS;
        long actual_time = current_time.tv_sec * 1e9 + current_time.tv_nsec;
        long jitter = actual_time - expected_time;
        
        csv_file << i + 1 << "," << expected_time << "," << actual_time << "," << jitter << "\n";
        
        if (DEBUG) {
            std::cout << "[DEBUG] Iteration " << i + 1 
                      << " | Expected: " << expected_time 
                      << " ns | Actual: " << actual_time 
                      << " ns | Jitter: " << jitter << " ns\n";
        }

        evl_sleep_until(timer_fd, &current_time);
    }

    csv_file.close(); // Close the CSV file

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
