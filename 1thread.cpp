#include <iostream>
#include <pthread.h>
#include <evl/thread.h>
#include <sched.h>
#include <unistd.h>
#include <cstring>
#include <fstream>
#include <time.h>

#define CPU_CORE 1  // Assign thread to core 1

std::ofstream log_file("execution_log.csv");

// Real-time thread function
void* rt_thread(void* arg) {
    std::cout << "[INFO] Real-time thread initializing...\n";

    // Attach the thread to Xenomai (EVL)
    struct evl_sched_attrs attrs;
    memset(&attrs, 0, sizeof(attrs));
    attrs.sched_policy = SCHED_FIFO;
    attrs.sched_priority = 80;

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

    std::cout << "[INFO] Real-time thread running at full speed on core " << CPU_CORE << std::endl;


    log_file << "Timestamp (s),Timestamp (ns),Jitter (ns)\n";

    struct timespec prev_time, now;
    clock_gettime(CLOCK_MONOTONIC, &prev_time);

    int i = 0;
    while (i < 5000) {
        clock_gettime(CLOCK_MONOTONIC, &now);

        long jitter_ns = (now.tv_sec - prev_time.tv_sec) * 1e9 + (now.tv_nsec - prev_time.tv_nsec);
        prev_time = now;

        // Simulate some computation (example: processing sensor data)
        volatile int x = 0;
        for (int i = 0; i < 10000; ++i) {
            x += i;
        }

        // Log execution time and jitter
        log_file << now.tv_sec << "," << now.tv_nsec << "," << jitter_ns << "\n";

        std::cout << "[INFO] Real-time thread executing task at " 
                  << now.tv_sec << "." << now.tv_nsec 
                  << " with jitter " << jitter_ns << " ns" << std::endl;
        i++;
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

    pthread_join(thread, nullptr);

    log_file.close();
    return EXIT_SUCCESS;
}
