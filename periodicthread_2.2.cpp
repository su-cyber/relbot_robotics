#include <iostream>
#include <fstream>
#include <pthread.h>
#include <evl/clock.h>
#include <evl/thread.h>
#include <evl/timer.h>
#include <cstring>  // For memset
#include <errno.h>  // For strerror
#include <unistd.h> // For usleep()

#define PERIOD_NS 1000000  // 1ms period
#define NUM_ITERATIONS 100  // Reduce iterations for testing
#define NUM_THREADS 2  // Limit to 2 threads for Raspberry Pi stability
#define DEBUG 1  // Enable (1) or Disable (0) Debug Messages

// Function for logging system state (optional)
void log_system_stats() {
    std::ofstream log("system_log.txt", std::ios::app);
    log << "Running test at: " << time(NULL) << std::endl;
    log.close();
    if (DEBUG) std::cout << "[DEBUG] System state logged.\n";
}

// Periodic real-time task
void *periodic_task(void *arg) {
    int thread_id = *(int *)arg;
    delete (int*)arg;  // Free allocated memory immediately after use

    std::string thread_name = "rt_thread_" + std::to_string(thread_id);
    if (DEBUG) std::cout << "[DEBUG] Thread " << thread_id << " initialized.\n";

    // Attach thread to EVL with Round Robin scheduling
    struct evl_sched_attrs attrs;
    memset(&attrs, 0, sizeof(attrs));
    attrs.sched_policy = SCHED_RR;  // Use Round Robin scheduling
    attrs.sched_priority = 5;  // Lower priority for fairness
    attrs.sched_rr_quantum = 2000000;  // 2ms time slice (quantum)

    int ret = evl_attach_thread(EVL_CLONE_PUBLIC, thread_name.c_str(), &attrs);
    if (ret < 0) {
        std::cerr << "[ERROR] Thread " << thread_id << " failed to attach: " << strerror(-ret) << "\n";
        return NULL;
    }
    if (DEBUG) std::cout << "[DEBUG] Thread " << thread_id << " attached to EVL with Round Robin scheduling.\n";

    struct timespec next_time;
    evl_read_clock(EVL_CLOCK_MONOTONIC, &next_time);
    if (DEBUG) std::cout << "[DEBUG] Thread " << thread_id << " starting periodic loop.\n";

    for (int i = 0; i < NUM_ITERATIONS; ++i) {
        // Get execution start time
        struct timespec start_time;
        evl_read_clock(EVL_CLOCK_MONOTONIC, &start_time);
        if (DEBUG) std::cout << "[DEBUG] Thread " << thread_id << " Iteration " << i 
                             << " Start Time: " << start_time.tv_sec << "." << start_time.tv_nsec << "\n";

        // Simulate CPU Load (Optional)
        volatile int x = 0;
        for (int j = 0; j < 10000; ++j) {
            x += j;
        }

        // Sleep until next period
        next_time.tv_nsec += PERIOD_NS;
        while (next_time.tv_nsec >= 1000000000) {
            next_time.tv_sec++;
            next_time.tv_nsec -= 1000000000;
        }
        evl_sleep_until(EVL_CLOCK_MONOTONIC, &next_time);
        if (DEBUG) std::cout << "[DEBUG] Thread " << thread_id << " Iteration " << i 
                             << " Next Wake-up Time: " << next_time.tv_sec << "." << next_time.tv_nsec << "\n";

        // Log execution time (Optional)
        std::ofstream log("execution_times.csv", std::ios::app);
        log << i << "," << (start_time.tv_nsec / 1e6) << "\n";
        log.close();
        if (DEBUG) std::cout << "[DEBUG] Thread " << thread_id << " Iteration " << i << " logged.\n";

        std::cout << "Thread " << thread_id << " - Iteration " << i << std::endl;
    }

    if (DEBUG) std::cout << "[DEBUG] Thread " << thread_id << " completed execution.\n";
    return NULL;
}

int main() {
    log_system_stats();  // Log initial state

    pthread_t threads[NUM_THREADS];

    // Attach main thread to EVL
    if (evl_attach_self("main_thread") < 0) {
        std::cerr << "[ERROR] Failed to attach main thread to EVL\n";
        return 1;
    }
    if (DEBUG) std::cout << "[DEBUG] Main thread attached to EVL.\n";

    // Create multiple threads safely
    for (int i = 0; i < NUM_THREADS; ++i) {
        int *tid = new int(i);  // Allocate a unique thread ID dynamically
        if (DEBUG) std::cout << "[DEBUG] Creating thread " << i << "\n";
        if (pthread_create(&threads[i], NULL, periodic_task, tid)) {
            std::cerr << "[ERROR] Error creating thread " << i << "\n";
            return 1;
        }
        usleep(5000); // Small delay to prevent overload during creation
    }

    // Wait for all threads to finish
    for (int i = 0; i < NUM_THREADS; ++i) {
        pthread_join(threads[i], NULL);
        if (DEBUG) std::cout << "[DEBUG] Thread " << i << " joined.\n";
    }

    std::cout << "Test completed!\n";
    if (DEBUG) std::cout << "[DEBUG] All threads finished execution.\n";
    return 0;
}
