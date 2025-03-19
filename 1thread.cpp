#include <iostream>
#include <pthread.h>
#include <evl/thread.h>
#include <evl/timer.h>
#include <sched.h>
#include <unistd.h>
#include <time.h>
#include <cstring>

#define PERIOD_NS 1000000  // 1ms period
#define CPU_CORE 1         // Assign thread to core 1

// Real-time thread function
void* rt_thread(void* arg) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);  // Get current time

    std::cout << "[INFO] Real-time thread initializing...\n";

    // Attach the thread to Xenomai's EVL real-time domain
    struct evl_sched_attrs attrs;
    memset(&attrs, 0, sizeof(attrs));
    attrs.sched_policy = SCHED_FIFO;
    attrs.sched_priority = 80;  // Set priority

    int ret = evl_attach_thread(EVL_CLONE_PUBLIC, "rt_thread", &attrs);
    if (ret < 0) {
        std::cerr << "[ERROR] evl_attach_thread failed: " << strerror(-ret) << "\n";
        return nullptr;
    }

    std::cout << "[INFO] Thread successfully attached to Xenomai (EVL)\n";

    // Set CPU affinity to Core 1 (Xenomai Core)
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(CPU_CORE, &cpuset);
    if (sched_setaffinity(0, sizeof(cpu_set_t), &cpuset) < 0) {
        std::cerr << "[ERROR] Failed to set CPU affinity to Core 1: " << strerror(errno) << "\n";
    } else {
        std::cout << "[INFO] Thread assigned to Core 1\n";
    }

    std::cout << "[INFO] Real-time thread started on core " << CPU_CORE << std::endl;

    while (true) {
        ts.tv_nsec += PERIOD_NS;
        if (ts.tv_nsec >= 1000000000) { // Handle overflow
            ts.tv_nsec -= 1000000000;
            ts.tv_sec++;
        }

        int ret = evl_usleep(PERIOD_NS / 1000);  // Sleep in microseconds
        if (ret) {
            std::cerr << "[ERROR] evl_usleep failed: " << strerror(-ret) << std::endl;
            break;
        }

        std::cout << "[INFO] Real-time thread executing on core " << CPU_CORE
                  << " at time " << ts.tv_sec << "." << ts.tv_nsec << std::endl;
    }

    return nullptr;
}

int main() {
    pthread_t thread;

    // Attach the main thread to EVL to ensure Xenomai scheduling
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
