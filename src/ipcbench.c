
#ifdef HAVE_CONFIG
#include "config.h"
#endif //HAVE_CONFIG

#include <ipcbench.h>

double overhead = 0;

#define STACK_SIZE (8*1024)

static double ticks_delta(struct timespec t0, struct timespec t1) {
    double dsec =  ((double)t1.tv_sec -  (double)t0.tv_sec);
    double dnsec = ((double)t1.tv_nsec - (double)t0.tv_nsec) / 1e9;
    return dsec + dnsec - overhead;
}

static struct timespec get_ticks() {
    struct timespec t;
    clock_gettime( CLOCK_MONOTONIC, &t );
    return t;
}

static void calibrate(void) {
    /* warm up */
    for( size_t i = 0; i < 10; i++) get_ticks();

    /* Average tick time */
    double a = 0;
    struct timespec r0,r1;
    overhead = 0;
    for( size_t i = 0; i<1000; i++ ) {
        r0 = get_ticks();
        r1 = get_ticks();
        a += ticks_delta(r0,r1);
    }
    overhead = (a) / 1000;
}


static void make_realtime( int priority ) {
    char prefault[STACK_SIZE];
    memset( prefault,0,sizeof(prefault) );

    if( mlockall( MCL_CURRENT | MCL_FUTURE ) ) {
        fprintf(stderr, "Couldn't lock pages in memory: %s\n",
                strerror(errno) );
        abort();
    }
    struct sched_param sp;
    sp.sched_priority = priority; /* 99 is max priority on linux */
    if( sched_setscheduler( 0, SCHED_FIFO, &sp) < 0 ) {
        fprintf(stderr, "Couldn't set scheduling priority: %s\n",
                strerror(errno) );
        abort();
    }
}


static void send(struct ipcbench_vtab *vtab) {
    vtab->init_send();

    while(1) {
        struct timespec ts = get_ticks();
        clock_gettime( CLOCK_MONOTONIC, &ts );
        vtab->send(&ts);
        sleep(1);
    }

    vtab->destroy_send();
}

static void recv(struct ipcbench_vtab *vtab) {
    vtab->init_recv();

    while(1) {
        struct timespec ts;
        vtab->recv(&ts);
        struct timespec now = get_ticks();
        printf("delta: %lf us\n", ticks_delta( ts, now ) * 1e6);
    }

    vtab->destroy_recv();
}

int main( int argc, char **argv ) {
    (void)argc;
    (void)argv;

    make_realtime(30);
    calibrate();
    fprintf(stderr, "overhead: %f us\n", overhead * 1e6);


    struct ipcbench_vtab *vtab = &ipc_bench_vtab_lcm;
    //struct ipcbench_vtab *vtab = &ipc_bench_vtab_ach;

    if( vtab->init ) vtab->init();

    pid_t pid_send = fork();
    if( 0 == pid_send ) {
        send(vtab);
        return 0;
    } else if ( pid_send < 0 ) {
        abort();
    }

    pid_t pid_listen = fork();
    if( 0 == pid_listen ) {
        recv(vtab);
        return 0;
    } else if ( pid_listen < 0 ) {
        abort();
    }

    sleep(100);


    if( vtab->destroy ) vtab->destroy();

    return 0;

}
