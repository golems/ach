
#include <stdio.h>

#include <inttypes.h>
#include <unistd.h>
#include <time.h>
#include <lcm/lcm.h>
#include "ipcbench_lcm_timestamp_t.h"

double overhead = 0;

static double ticks_delta(struct timespec t0, struct timespec t1) {
    double dsec =  ((double)t1.tv_sec -  (double)t0.tv_sec);
    double dnsec = ((double)t1.tv_nsec - (double)t0.tv_nsec) / 1e9;
    return dsec + dnsec - overhead;
}

struct timespec get_ticks() {
    struct timespec t;
    clock_gettime( CLOCK_MONOTONIC, &t );
    return t;
}

void calibrate(void) {
    //make_realtime(30);
    double a = 0;
    struct timespec r0,r1;
    size_t i;
    overhead = 0;
    for( i = 0; i<1000; i++ ) {
        r0 = get_ticks();
        r1 = get_ticks();
        a += ticks_delta(r0,r1);
    }
    overhead = (a) / 1000;
}



void send(void)
{
    lcm_t * lcm = lcm_create(NULL);
    if(!lcm)
        abort();

    ipcbench_lcm_timestamp_t msg;
    msg.secs = 0;
    msg.nsecs = 0;

    while(1) {
        struct timespec ts;
        clock_gettime( CLOCK_MONOTONIC, &ts );
        msg.secs = ts.tv_sec;
        msg.nsecs = ts.tv_nsec;
        ipcbench_lcm_timestamp_t_publish(lcm, "EXAMPLE", &msg);
        sleep(1);
    }

    lcm_destroy(lcm);
}


static void
listen_handler(const lcm_recv_buf_t *rbuf, const char * channel,
        const ipcbench_lcm_timestamp_t * msg, void * user)
{

    struct timespec ts0 = {.tv_sec = msg->secs, .tv_nsec = msg->nsecs };
    struct timespec ts1;
    clock_gettime( CLOCK_MONOTONIC, &ts1 );
    (void)rbuf;
    (void)user;
    printf("Received message on channel \"%s\":\n", channel);
    printf("secs: %ld, nsecs: %d\n", msg->secs, msg->nsecs );
    printf("delta: %lf us\n", ticks_delta( ts0, ts1 ) * 1e6);


    /* printf("  timestamp   = %"PRId64"\n", msg->timestamp); */
    /* printf("  position    = (%f, %f, %f)\n", */
    /*         msg->position[0], msg->position[1], msg->position[2]); */
    /* printf("  orientation = (%f, %f, %f, %f)\n", */
    /*         msg->orientation[0], msg->orientation[1], msg->orientation[2], */
    /*         msg->orientation[3]); */
    /* printf("  ranges:"); */
    /* for(i = 0; i < msg->num_ranges; i++) */

    /*     printf(" %d", msg->ranges[i]); */
    /* printf("\n"); */
    /* printf("  name        = '%s'\n", msg->name); */
    /* printf("  enabled     = %d\n", msg->enabled); */
}


void listen(void)
{
    lcm_t * lcm = lcm_create(NULL);
    if(!lcm)
        abort();

    ipcbench_lcm_timestamp_t_subscribe(lcm, "EXAMPLE", &listen_handler, NULL);

    while(1)
        lcm_handle(lcm);

    lcm_destroy(lcm);
    return;
}


int main( int argc, char **argv ) {
    (void)argc;
    (void)argv;

    calibrate();
    printf("overhead: %f\n", overhead);

    pid_t pid_send = fork();
    if( 0 == pid_send ) {
        send();
        return 0;
    } else if ( pid_send < 0 ) {
        abort();
    }

    pid_t pid_listen = fork();
    if( 0 == pid_listen ) {
        listen();
        return 0;
    } else if ( pid_listen < 0 ) {
        abort();
    }

    sleep(100);
    return 0;
}
