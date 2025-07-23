#include <limits.h>
#include <pthread.h>
#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>

#include <sys/stat.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <iostream>
#define PERIOD_NS 200000
#define SEC_IN_NSEC 1000000000

static int latency_target_fd = -1;
static int32_t latency_target_value = 0;
static void set_latency_target(void)
{
    struct stat s;
    int err;
    int errno;
    errno = 0;
    err = stat("/dev/cpu_dma_latency", &s);
    if (err == -1)
    {
        std::cout<<"WARN: stat /dev/cpu_dma_latency failed"<<std::endl;
        return;
    }

    errno = 0;
    latency_target_fd = open("/dev/cpu_dma_latency", O_RDWR);
    if (latency_target_fd == -1)
    {
        std::cout<<"WARN: open /dev/cpu_dma_latency"<<std::endl;
        return;
    }

    errno = 0;
    err = write(latency_target_fd, &latency_target_value, 4);
    if (err < 1)
    {
        std::cout<<"# error setting cpu_dma_latency to %d!"<<latency_target_value<<std::endl;
        close(latency_target_fd);
        return;
    }
    printf("# /dev/cpu_dma_latency set to %dus\n", latency_target_value);
}

void *rt_thread(void *data)
{
    struct timespec ts;
    struct timespec ts2;
    clock_gettime(CLOCK_MONOTONIC, &ts);

    int max = 0;
    int min = 1000000;
    int avg = 0;
    int total = 0;
    int latency = 0;
    int t_cnt = 0;

    ts.tv_sec += 1;
    while (t_cnt < 100000)
    {
        t_cnt++;
        ts.tv_nsec += PERIOD_NS;
        while (ts.tv_nsec >= SEC_IN_NSEC)
        {
            ts.tv_sec++;
            ts.tv_nsec -= SEC_IN_NSEC;
        }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL);
        clock_gettime(CLOCK_MONOTONIC, &ts2);

        int latency = ts2.tv_nsec - ts.tv_nsec;
        if (latency < 0)
        {
            latency += SEC_IN_NSEC;
        }

        if (max < latency)
            max = latency;

        if (min > latency)
            min = latency;

        total += latency;
        avg = total / t_cnt;
    }
    std::cout << "avg : " << avg / 1000.0 << "  max : " << max / 1000.0<< " min : "<<min / 1000.0 << std::endl;
    return NULL;
}

int main(int argc, char *argv[])
{
    struct sched_param param;
    pthread_attr_t attr;
    pthread_t thread;
    int ret;
    //set_latency_target();

    /* Initialize pthread attributes (default values) */
    ret = pthread_attr_init(&attr);
    if (ret)
    {
        printf("init pthread attributes failed\n");
        goto out;
    }

    /* Set scheduler policy and priority of pthread */
    ret = pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
    if (ret)
    {
        printf("pthread setschedpolicy failed\n");
        goto out;
    }
    param.sched_priority = 80;
    ret = pthread_attr_setschedparam(&attr, &param);
    if (ret)
    {
        printf("pthread setschedparam failed\n");
        goto out;
    }
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(2, &cpuset);

    ret = pthread_attr_setaffinity_np(&attr, sizeof(cpuset), &cpuset);
    if (ret)
    {
        printf("pthread setaffinity failed\n");
        goto out;
    }
    /* Use scheduling parameters of attr */
    ret = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
    if (ret)
    {
        printf("pthread setinheritsched failed\n");
        goto out;
    }

    /* Create a pthread with specified attributes */
    ret = pthread_create(&thread, &attr, rt_thread, NULL);
    if (ret)
    {
        printf("create pthread failed\n");
        goto out;
    }

    /* Join the thread and wait until it is done */
    ret = pthread_join(thread, NULL);
    if (ret)
        printf("join pthread failed: %m\n");

out:
    return ret;
}
