#include <iostream>
#include <fstream>
#include <pthread.h>
#include <evl/clock.h>
#include <evl/thread.h>
#include <evl/timer.h>
#include <cstring>  // For memset
#include <errno.h>  // For strerror
#include <unistd.h> // For usleep()
#include <sched.h>  // For CPU affinity
#include <atomic>   // For thread synchronization

#define PERIOD_NS 1000000  // 1ms period
#define NUM_ITERATIONS 100
#define NUM_THREADS 2  // Limit to 2 threads for stability
#define DEBUG 1  // Enable (1) or Disable (0) Debug Messages

std::atomic<int> threads_ready(0); // Atomic counter for thread synchronization

// Function for logging system state
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

    // Attach thread to EVL with FIFO scheduling
    struct evl_sched_attrs attrs;
    memset(&attrs, 0, sizeof(attrs));
    attrs.sched_policy = SCHED_FIFO;  // Use FIFO instead of Round Robin
    attrs.sched_priority = 10;  // Higher priority to prevent delays

    int ret = evl_attach_thread(EVL_CLONE_PUBLIC, thread_name.c_str(), &attrs);
    if (ret < 0) {
        std::cerr << "[ERROR] Thread " << thread_id << " failed to attach: " << strerror(-ret) << "\n";
        return NULL;
    }
    if (DEBUG) std::cout << "[DEBUG] Thread " << thread_id << " attached to EVL with FIFO scheduling.\n";

    // Manually pin thread to an available core (0, 2, or 3, avoiding Xenomai's Core 1)
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    int core_assignment[3] = {0, 2, 3}; // Allowed cores
    int core = core_assignment[thread_id % 3]; // Distribute across cores 0, 2, 3
    CPU_SET(core, &cpuset);
    ret = sched_setaffinity(0, sizeof(cpu_set_t), &cpuset);
    if (ret < 0) {
        std::cerr << "[ERROR] Thread " << thread_id << " failed to set CPU affinity: " << strerror(errno) << "\n";
    } else {
        if (DEBUG) std::cout << "[DEBUG] Thread " << thread_id << " assigned to Core " << core << "\n";
    }

    struct timespec next_time;
    evl_read_clock(EVL_CLOCK_MONOTONIC, &next_time);

    // Synchronize threads before execution
    threads_ready.fetch_add(1);
    while (threads_ready.load() < NUM_THREADS) {
        usleep(100);  // Wait for all threads to be ready
    }

    if (DEBUG) std::cout << "[DEBUG] Thread " << thread_id << " starting periodic loop.\n";

    // Open a separate CSV file for each thread
    std::string filename = "execution_thread_" + std::to_string(thread_id) + ".csv";
    std::ofstream log(filename);
    if (!log.is_open()) {
        std::cerr << "[ERROR] Could not open file: " << filename << "\n";
        return NULL;
    }
    log << "Iteration,ExecutionTime(ms)\n"; // Write CSV header

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
        next_time.tv_sec = start_time.tv_sec;
        next_time.tv_nsec = start_time.tv_nsec + PERIOD_NS;
        while (next_time.tv_nsec >= 1000000000) {
            next_time.tv_sec++;
            next_time.tv_nsec -= 1000000000;
        }

        evl_sleep_until(EVL_CLOCK_MONOTONIC, &next_time);
        if (DEBUG) std::cout << "[DEBUG] Thread " << thread_id << " Iteration " << i 
                             << " Next Wake-up Time: " << next_time.tv_sec << "." << next_time.tv_nsec << "\n";

        // Calculate execution time in ms
        double elapsed_time_ms = (next_time.tv_sec - start_time.tv_sec) * 1e3 +
                                 (next_time.tv_nsec - start_time.tv_nsec) / 1e6;

        // Write execution time to the thread's CSV file
        log << i << "," << elapsed_time_ms << "\n";
        if (DEBUG) std::cout << "[DEBUG] Thread " << thread_id << " Iteration " << i << " logged in " << filename << "\n";

        std::cout << "Thread " << thread_id << " - Iteration " << i << std::endl;
    }

    log.close(); // Close the file
    if (DEBUG) std::cout << "[DEBUG] Thread " << thread_id << " completed execution and saved data to " << filename << "\n";
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
