#include <iostream>
#include <pthread.h>
#include <evl/thread.h>
#include <evl/timer.h>
#include <sched.h>
#include <unistd.h>
#include <time.h>

#define PERIOD_NS 1000000  // 1ms period
#define CPU_CORE 1          // Assign thread to core 1

// Function for the real-time thread
void* rt_thread(void* arg) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);  // Get current time

    std::cout << "Real-time thread started on core " << CPU_CORE << std::endl;

    while (true) {
        ts.tv_nsec += PERIOD_NS;
        if (ts.tv_nsec >= 1000000000) { // Handle overflow
            ts.tv_nsec -= 1000000000;
            ts.tv_sec++;
        }

        int ret = evl_usleep(PERIOD_NS / 1000);  // Sleep in microseconds
        if (ret) {
            std::cerr << "evl_usleep failed: " << strerror(-ret) << std::endl;
            break;
        }

        std::cout << "Real-time thread executing on core " << CPU_CORE
                  << " at time " << ts.tv_sec << "." << ts.tv_nsec << std::endl;
    }

    return nullptr;
}

int main() {
    pthread_t thread;
    cpu_set_t cpuset;
    struct sched_param param;

    // Initialize CPU set and bind the thread to core 1
    CPU_ZERO(&cpuset);
    CPU_SET(CPU_CORE, &cpuset);

    // Create the real-time thread
    if (pthread_create(&thread, nullptr, rt_thread, nullptr)) {
        std::cerr << "Error: pthread_create failed!" << std::endl;
        return EXIT_FAILURE;
    }

    // Set CPU affinity to core 1
    if (pthread_setaffinity_np(thread, sizeof(cpu_set_t), &cpuset)) {
        std::cerr << "Error: pthread_setaffinity_np failed!" << std::endl;
        return EXIT_FAILURE;
    }

    // Set real-time priority
    param.sched_priority = 80;  // Priority 80 for high-priority real-time tasks
    if (pthread_setschedparam(thread, SCHED_FIFO, &param)) {
        std::cerr << "Error: pthread_setschedparam failed!" << std::endl;
        return EXIT_FAILURE;
    }

    // Wait for the thread to finish (won't happen in a periodic loop)
    pthread_join(thread, nullptr);
    return EXIT_SUCCESS;
}
