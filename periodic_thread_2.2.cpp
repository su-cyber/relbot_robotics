#include <iostream>
#include <pthread.h>
#include <time.h>
#include <unistd.h>
#include <fstream>

#define PERIOD_NS 1000000  // 1ms = 1,000,000 ns
#define NUM_ITERATIONS 1000  // Run for 1000 iterations
#define NUM_THREADS 3  // Number of periodic threads

struct ThreadData {
    int id;
    std::ofstream file;
};

void *periodic_task(void *arg) {
    ThreadData *data = (ThreadData *)arg;
    struct timespec next_time;
    clock_gettime(CLOCK_MONOTONIC, &next_time); // Get current time

    // Introduce different start times for each thread
    usleep(data->id * 1000);

    for (int i = 0; i < NUM_ITERATIONS; ++i) {
        // Dummy computation to simulate workload
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

        // Write to CSV file
        data->file << i << "," << elapsed_time << std::endl;
    }

    data->file.close();
    return NULL;
}

int main() {
    pthread_t threads[NUM_THREADS];
    ThreadData threadData[NUM_THREADS];

    // Create multiple periodic threads
    for (int i = 0; i < NUM_THREADS; ++i) {
        threadData[i].id = i;
        threadData[i].file.open("execution_times_thread_" + std::to_string(i) + ".csv");
        threadData[i].file << "Iteration,ExecutionTime(ms)" << std::endl;

        if (pthread_create(&threads[i], NULL, periodic_task, &threadData[i])) {
            std::cerr << "Error creating thread " << i << std::endl;
            return 1;
        }
    }

    // Wait for all threads to finish
    for (int i = 0; i < NUM_THREADS; ++i) {
        pthread_join(threads[i], NULL);
    }

    std::cout << "All periodic tasks finished!" << std::endl;
    return 0;
}
