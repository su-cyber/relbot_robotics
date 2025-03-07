#include <iostream>
#include <pthread.h>
#include <time.h>
#include <unistd.h>
#include <fstream>

#define PERIOD_NS 1000000  // 1ms = 1,000,000 ns
#define NUM_ITERATIONS 1000  // Run for 1000 iterations
#define OUTPUT_FILE "clock_monitor.csv"  // Output CSV file name

void *monitor_clock(void *arg) {
    std::ofstream file(OUTPUT_FILE);
    if (!file.is_open()) {
        return NULL; // File open error, exit thread
    }

    // CSV Header
    file << "Iteration,ExecutionTime(ms),ActualPeriod(ms),Jitter(ms)\n";

    struct timespec next_time, start_time, end_time, last_time;
    clock_gettime(CLOCK_MONOTONIC, &next_time);
    last_time = next_time;

    for (int i = 0; i < NUM_ITERATIONS; ++i) {
        // Measure start time
        clock_gettime(CLOCK_MONOTONIC, &start_time);

        // Simulate workload
        volatile int x = 0;
        for (int j = 0; j < 1000; ++j) {
            x += j;
        }

        // Sleep until the next period
        next_time.tv_nsec += PERIOD_NS;
        while (next_time.tv_nsec >= 1000000000) {
            next_time.tv_sec++;
            next_time.tv_nsec -= 1000000000;
        }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL);

        // Measure end time
        clock_gettime(CLOCK_MONOTONIC, &end_time);

        // Compute execution time
        double exec_time = (end_time.tv_sec - start_time.tv_sec) * 1e3 +
                           (end_time.tv_nsec - start_time.tv_nsec) / 1e6;

        // Compute actual period
        double actual_period = (end_time.tv_sec - last_time.tv_sec) * 1e3 +
                               (end_time.tv_nsec - last_time.tv_nsec) / 1e6;

        // Compute jitter (deviation from ideal 1ms period)
        double jitter = actual_period - (PERIOD_NS / 1e6);

        // Store last wake-up time
        last_time = end_time;

        // Write to CSV file
        file << i << "," << exec_time << "," << actual_period << "," << jitter << "\n";
    }

    file.close();
    return NULL;
}

int main() {
    pthread_t thread;
    
    if (pthread_create(&thread, NULL, monitor_clock, NULL)) {
        return 1; // Thread creation failed
    }

    pthread_join(thread, NULL);
    return 0;
}
