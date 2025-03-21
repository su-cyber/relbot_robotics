#include <pthread.h>
#include <evl/thread.h>
#include <evl/sched.h>
#include <unistd.h>
#include <cstring>
#include <time.h>
#include <evl/clock.h>
#include <evl/proxy.h>
#include <fstream>

#define CPU_CORE 1  // Core reserved for real-time execution
#define NUM_ITERATIONS 5000  

// Structure to store execution timestamps and jitter values
struct JitterEntry {
    long sec;       
    long nsec;      
    long jitter_ns; 
};

JitterEntry jitter_log[NUM_ITERATIONS];  // In-memory log storage

// Real-time thread function
void* rt_thread(void* arg) {
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(CPU_CORE, &cpuset);
    
    // Bind the thread to Core 1 for isolation
    pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);

    // Attach the thread to Xenomai EVL real-time scheduler
    if (!evl_attach_self("rt_thread"))
        return nullptr;

    struct timespec prev_time, now;
    evl_read_clock(EVL_CLOCK_MONOTONIC, &prev_time);  

    for (int i = 0; i < NUM_ITERATIONS; ++i) {
        evl_read_clock(EVL_CLOCK_MONOTONIC, &now);  
        
        // Compute jitter as the time difference between consecutive iterations
        long jitter = (now.tv_sec - prev_time.tv_sec) * 1e9 + (now.tv_nsec - prev_time.tv_nsec);
        prev_time = now;

        // Simulated workload to observe execution timing consistency
        volatile int x = 0;
        for (int j = 0; j < 10000; ++j) x += j;

        // Store the results in memory
        jitter_log[i] = {now.tv_sec, now.tv_nsec, jitter};
    }

    evl_detach_self(); 
    return nullptr;
}

int main() {
    pthread_t thread;
    pthread_attr_t attrs;
    sched_param params;
    
    params.sched_priority = 80;  
    
    pthread_attr_init(&attrs);
    pthread_attr_setinheritsched(&attrs, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setschedpolicy(&attrs, SCHED_FIFO);  // FIFO for deterministic scheduling
    pthread_attr_setschedparam(&attrs, &params);
    
    // Create and start the real-time thread
    pthread_create(&thread, &attrs, rt_thread, nullptr);
    pthread_join(thread, nullptr);  

    // Write recorded jitter data to CSV file for analysis
    std::ofstream csv("jitter_log.csv");
    csv << "Timestamp (s),Timestamp (ns),Jitter (ns)\n";
    for (int i = 0; i < NUM_ITERATIONS; ++i) {
        csv << jitter_log[i].sec << ","
            << jitter_log[i].nsec << ","
            << jitter_log[i].jitter_ns << "\n";
    }
    csv.close();

    return 0;
}
