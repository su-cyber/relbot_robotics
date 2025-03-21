#include <pthread.h>
#include <evl/thread.h>
#include <evl/sched.h>
#include <unistd.h>
#include <cstring>
#include <time.h>
#include <evl/clock.h>

#define CPU_CORE 1
#define NUM_ITERATIONS 1000000

struct JitterEntry {
    long sec;
    long nsec;
    long jitter_ns;
};
JitterEntry jitter_log[NUM_ITERATIONS];  // In-memory array

void* rt_thread(void* arg) {
    struct evl_sched_attrs attrs;
    memset(&attrs, 0, sizeof(attrs));

    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(CPU_CORE, &cpuset);
    pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);

    if (evl_attach_thread(EVL_CLONE_PUBLIC, "rt_thread", &attrs) < 0)
        return nullptr;

    struct timespec prev_time, now;
    evl_read_clock(EVL_CLOCK_MONOTONIC, &prev_time);

    for (int i = 0; i < NUM_ITERATIONS; ++i) {
        evl_read_clock(EVL_CLOCK_MONOTONIC, &now);
        long jitter = (now.tv_sec - prev_time.tv_sec) * 1e9 + (now.tv_nsec - prev_time.tv_nsec);
        prev_time = now;

        // Simulated workload
        volatile int x = 0;
        for (int j = 0; j < 10000; ++j) x += j;

        jitter_log[i] = {now.tv_sec, now.tv_nsec, jitter};
    }
    return nullptr;
}

int main() {
    pthread_t thread;
    pthread_attr_t attrs;
    sched_param params;
    params.sched_priority = 80;
    pthread_attr_init(&attrs);
    pthread_attr_setinheritsched(&attrs, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setschedpolicy(&attrs, SCHED_FIFO);
    pthread_attr_setschedparam(&my_attr, &params);
    pthread_create(&thread, nullptr, rt_thread, nullptr);
    pthread_join(thread, nullptr);
    return 0;
}
