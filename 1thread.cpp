#include <iostream>
#include <pthread.h>
#include <evl/thread.h>
#include <sched.h>
#include <unistd.h>
#include <cstring>

#define CPU_CORE 1  // Assign thread to core 1

// Real-time thread function
void* rt_thread(void* arg) {
    // Attach the thread to EVL with FIFO scheduling
    struct evl_sched_attrs attrs;
    memset(&attrs, 0, sizeof(attrs));
    attrs.sched_policy = SCHED_FIFO;
    attrs.sched_priority = 80;

    int ret = evl_attach_thread(EVL_CLONE_PUBLIC, "rt_thread", &attrs);
    if (ret < 0) return nullptr;

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

    // Run continuously without sleep
    while (true) {
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);

        // Simulate workload
        volatile int x = 0;
        for (int i = 0; i < 10000; ++i) {
            x += i;
        }

        std::cout << "[INFO] Real-time thread executing task at " 
                  << now.tv_sec << "." << now.tv_nsec << std::endl;
    }

    return nullptr;
}

int main() {
    pthread_t thread;

    // Create the real-time thread
    if (pthread_create(&thread, nullptr, rt_thread, nullptr)) {
        return EXIT_FAILURE;
    }

    pthread_join(thread, nullptr);

    // Dump jitter data to CSV
    std::ofstream log_file("execution_log.csv");
    log_file << "Timestamp (s),Timestamp (ns),Jitter (ns)\n";
    for (const auto& entry : jitter_log) {
        log_file << entry.sec << "," << entry.nsec << "," << entry.jitter_ns << "\n";
    }
    log_file.close();

    log_file.close();
    return EXIT_SUCCESS;
}
