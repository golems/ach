extern double overhead;
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
    size_t i;
    for( i = 0; i < 10; i++) get_ticks();

    /* Average tick time */
    double a = 0;
    struct timespec r0,r1;
    overhead = 0;
    for( i = 0; i<1000; i++ ) {
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
        perror( "Couldn't lock pages in memory");
    }
    struct sched_param sp;
    sp.sched_priority = priority; /* 99 is max priority on linux */
    if( sched_setscheduler( 0, priority ? SCHED_FIFO : SCHED_OTHER, &sp) < 0 ) {
        perror("Couldn't set scheduling priority");
    }
}
