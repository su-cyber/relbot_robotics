#include <iostream>
#include <pthread.h>
#include <evl/thread.h>
#include <evl/timer.h>
#include <evl/clock.h>
#include <sched.h>
#include <unistd.h>
#include <cstring>

#define PERIOD_NS 1000000  // 1ms period in nanoseconds
#define CPU_CORE 1         // Assign thread to core 1

// Real-time thread function
void* rt_thread(void* arg) {
    int timer_fd;
    
    std::cout << "[INFO] Real-time thread initializing...\n";

    // Attach thread to Xenomai (EVL)
    struct evl_sched_attrs attrs;
    memset(&attrs, 0, sizeof(attrs));
    attrs.sched_policy = SCHED_FIFO;
    attrs.sched_priority = 80;  // High priority

    int ret = evl_attach_thread(EVL_CLONE_PUBLIC, "rt_thread", &attrs);
    if (ret < 0) {
        std::cerr << "[ERROR] evl_attach_thread failed: " << strerror(-ret) << "\n";
        return nullptr;
    }
    std::cout << "[INFO] Thread successfully attached to Xenomai (EVL)\n";

    // Set CPU affinity to Core 1
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(CPU_CORE, &cpuset);
    if (pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset) < 0) {
        std::cerr << "[ERROR] Failed to set CPU affinity to Core 1: " << strerror(errno) << "\n";
    } else {
        std::cout << "[INFO] Thread assigned to Core 1\n";
    }

    // Open an EVL timer
    timer_fd = evl_open_timer(EVL_CLOCK_MONOTONIC, "rt_timer");
    if (timer_fd < 0) {
        std::cerr << "[ERROR] Failed to open EVL timer: " << strerror(-timer_fd) << "\n";
        return nullptr;
    }

    // Make the timer periodic
    ret = evl_set_periodic(timer_fd, PERIOD_NS);
    if (ret < 0) {
        std::cerr << "[ERROR] evl_set_periodic failed: " << strerror(-ret) << "\n";
        return nullptr;
    }

    std::cout << "[INFO] Real-time thread started on core " << CPU_CORE << std::endl;

    while (true) {
        // Wait for the next period
        ret = evl_wait_period(timer_fd);
        if (ret < 0) {
            std::cerr << "[ERROR] evl_wait_period failed: " << strerror(-ret) << "\n";
            break;
        }

        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        std::cout << "[INFO] Real-time thread executing on core " << CPU_CORE
                  << " at time " << now.tv_sec << "." << now.tv_nsec << std::endl;
    }

    return nullptr;
}

int main() {
    pthread_t thread;

    // Attach main thread to EVL
    if (evl_attach_self("main_thread") < 0) {
        std::cerr << "[ERROR] Failed to attach main thread to EVL\n";
        return EXIT_FAILURE;
    }
    std::cout << "[INFO] Main thread attached to EVL.\n";

    // Create the real-time thread
    if (pthread_create(&thread, nullptr, rt_thread, nullptr)) {
        std::cerr << "[ERROR] pthread_create failed!\n";
        return EXIT_FAILURE;
    }

    // Wait for the real-time thread to finish (infinite loop)
    pthread_join(thread, nullptr);

    return EXIT_SUCCESS;
}
