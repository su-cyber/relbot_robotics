#include <iostream>
#include <fstream>
#include <pthread.h>
#include <evl/clock.h>
#include <evl/thread.h>
#include <evl/timer.h>
#include <cstring>  // For memset
#include <errno.h>  // For strerror

#define PERIOD_NS 1000000  // 1ms period
#define NUM_ITERATIONS 100  // Keep low for testing

// Function for logging system state (optional)
void log_system_stats() {
    std::ofstream log("system_log.txt", std::ios::app);
    log << "Running test at: " << time(NULL) << std::endl;
    log.close();
}

// Periodic real-time task
void *periodic_task(void *arg) {
    int thread_id = *(int *)arg;
    std::string thread_name = "rt_thread_" + std::to_string(thread_id);

    // Attach thread to EVL
    struct evl_sched_attrs attrs;
    memset(&attrs, 0, sizeof(attrs));
    attrs.sched_policy = SCHED_OTHER;  // Start with normal scheduling
    attrs.sched_priority = 0;  // Low priority for now

    int ret = evl_attach_thread(EVL_CLONE_PUBLIC, thread_name.c_str(), &attrs);
    if (ret < 0) {
        std::cerr << "Failed to attach thread: " << strerror(-ret) << "\n";
        return NULL;
    }

    struct timespec next_time;
    evl_read_clock(EVL_CLOCK_MONOTONIC, &next_time);

    for (int i = 0; i < NUM_ITERATIONS; ++i) {
        // Get execution start time
        struct timespec start_time;
        evl_read_clock(EVL_CLOCK_MONOTONIC, &start_time);

        // ====================
        // STEP 2: ADD CPU LOAD
        // ====================
        // Uncomment this to test CPU stress
        /*
        volatile int x = 0;
        for (int j = 0; j < 50000; ++j) {
            x += j;
        }
        */

        // Sleep until next period
        next_time.tv_nsec += PERIOD_NS;
        while (next_time.tv_nsec >= 1000000000) {
            next_time.tv_sec++;
            next_time.tv_nsec -= 1000000000;
        }
        evl_sleep_until(EVL_CLOCK_MONOTONIC, &next_time);

        // ====================
        // STEP 3: ADD FILE LOGGING
        // ====================
        // Uncomment this to log execution times
        /*
        std::ofstream log("execution_times.csv", std::ios::app);
        log << i << "," << (start_time.tv_nsec / 1e6) << "\n";
        log.close();
        */

        std::cout << "Thread " << thread_id << " - Iteration " << i << std::endl;
    }

    return NULL;
}

int main() {
    log_system_stats();  // Log initial state

    pthread_t thread;
    int thread_id = 0;

    evl_attach_self("main_thread");

    // ====================
    // STEP 4: ADD MULTI-THREADING
    // ====================
    // Uncomment to test multiple threads
    /*
    #define NUM_THREADS 3
    pthread_t threads[NUM_THREADS];
    int thread_ids[NUM_THREADS];

    for (int i = 0; i < NUM_THREADS; ++i) {
        thread_ids[i] = i;
        pthread_create(&threads[i], NULL, periodic_task, &thread_ids[i]);
    }

    for (int i = 0; i < NUM_THREADS; ++i) {
        pthread_join(threads[i], NULL);
    }
    */

    // Run a single thread for now
    if (pthread_create(&thread, NULL, periodic_task, &thread_id)) {
        std::cerr << "Error creating thread\n";
        return 1;
    }

    pthread_join(thread, NULL);
    
    std::cout << "Test completed!\n";
    return 0;
}
