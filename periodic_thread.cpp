#include <iostream>
#include <pthread.h>
#include <time.h>
#include <unistd.h>
#include <fstream>

#define PERIOD_NS 1000000  // 1ms = 1,000,000 ns
#define NUM_ITERATIONS 1000  // Run for 1000 iterations
#define OUTPUT_FILE "execution_times.csv"  // Output CSV file name

void *periodic_task(void *arg) {
    std::ofstream file(OUTPUT_FILE); // Open file for writing
    if (!file.is_open()) {
        std::cerr << "Error opening file for writing!" << std::endl;
        return NULL;
    }

    // Write CSV header
    file << "Iteration,ExecutionTime(ms)\n";

    struct timespec next_time;
    clock_gettime(CLOCK_MONOTONIC, &next_time); // Get current time

    for (int i = 0; i < NUM_ITERATIONS; ++i) {
        // Perform some dummy computation (simulate workload)
        volatile int x = 0;
        for (int j = 0; j < 1000; ++j) {
            x += j;
        }

        // Get execution time
        struct timespec start_time, end_time;
        clock_gettime(CLOCK_MONOTONIC, &start_time);

        // Sleep until the next period
        next_time.tv_nsec += PERIOD_NS;
        while (next_time.tv_nsec >= 1000000000) {
            next_time.tv_sec++;
            next_time.tv_nsec -= 1000000000;
        }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL);

        // Measure actual execution time
        clock_gettime(CLOCK_MONOTONIC, &end_time);
        double elapsed_time = (end_time.tv_sec - start_time.tv_sec) * 1e3 +
                              (end_time.tv_nsec - start_time.tv_nsec) / 1e6;

        // Write execution time to file
        file << i << "," << elapsed_time << "\n";
    }

    file.close(); // Close the file properly
    return NULL;
}

int main() {
    pthread_t thread;
    
    // Create a real-time periodic thread
    if (pthread_create(&thread, NULL, periodic_task, NULL)) {
        std::cerr << "Error creating thread!" << std::endl;
        return 1;
    }

    // Wait for thread to finish
    pthread_join(thread, NULL);
    
    std::cout << "Execution times saved to " << OUTPUT_FILE << std::endl;
    return 0;
}
