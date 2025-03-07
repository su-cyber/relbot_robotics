#include <iostream>
#include <fstream>
#include <pthread.h>
#include <signal.h>
#include <time.h>
#include <unistd.h>

#define INTERVAL_NS 1000000  // 1.0 ms in nanoseconds
#define NUM_ITERATIONS 1000  // Number of iterations

timer_t timer_id;
std::ofstream csvFile("timing_data.csv");  // CSV file to store results
volatile bool timerTriggered = false;

// Signal handler for timer
void timer_handler(int sig, siginfo_t *si, void *uc) {
    (void)si;
    (void)uc;

    if (sig == SIGRTMIN) {
        timerTriggered = true; // Set flag when signal is received
    }
}

void *periodic_thread(void *arg) {
    struct timespec start, end;
    struct sigevent sev;
    struct itimerspec its;
    struct sigaction sa;

    // Set up signal handler
    sa.sa_flags = SA_SIGINFO;
    sa.sa_sigaction = timer_handler;
    sigemptyset(&sa.sa_mask);
    if (sigaction(SIGRTMIN, &sa, NULL) == -1) {
        perror("sigaction");
        return nullptr;
    }

    // Create the timer
    sev.sigev_notify = SIGEV_SIGNAL;
    sev.sigev_signo = SIGRTMIN;
    sev.sigev_value.sival_ptr = &timer_id;
    if (timer_create(CLOCK_MONOTONIC, &sev, &timer_id) == -1) {
        perror("timer_create");
        return nullptr;
    }

    // Set the timer to trigger every 1 ms
    its.it_value.tv_sec = 0;
    its.it_value.tv_nsec = INTERVAL_NS;
    its.it_interval.tv_sec = 0;
    its.it_interval.tv_nsec = INTERVAL_NS;
    if (timer_settime(timer_id, 0, &its, NULL) == -1) {
        perror("timer_settime");
        return nullptr;
    }

    // Write CSV Header
    if (csvFile.is_open()) {
        csvFile << "Iteration,Elapsed Time (ms)\n";
    }

    std::cout << "Periodic thread started, running every 1ms...\n";

    for (int i = 0; i < NUM_ITERATIONS; i++) {
        clock_gettime(CLOCK_MONOTONIC, &start);

        // Wait for signal to be received
        while (!timerTriggered) {
            usleep(100);  // Small sleep to avoid CPU overuse
        }
        timerTriggered = false;  // Reset flag

        // Simulate computational work
        volatile double sum = 0.0;
        for (int j = 0; j < 10000; j++) {
            sum += j * 0.01;
        }

        clock_gettime(CLOCK_MONOTONIC, &end);
        double elapsed_ms = (end.tv_sec - start.tv_sec) * 1000.0 + (end.tv_nsec - start.tv_nsec) / 1000000.0;

        // Write to CSV
        if (csvFile.is_open()) {
            csvFile << i + 1 << "," << elapsed_ms << "\n";
        }
    }

    // Cleanup
    timer_delete(timer_id);
    csvFile.close();
    return nullptr;
}

int main() {
    pthread_t thread;

    // Create the periodic thread
    if (pthread_create(&thread, nullptr, periodic_thread, nullptr) != 0) {
        perror("pthread_create");
        return EXIT_FAILURE;
    }

    // Wait for the thread to finish
    pthread_join(thread, nullptr);

    std::cout << "Execution completed. Data saved to timing_data.csv\n";
    return EXIT_SUCCESS;
}
