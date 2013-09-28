
#ifdef HAVE_CONFIG
#include "config.h"
#endif //HAVE_CONFIG

#include "ipcbench.h"
#include "util.h"




static void send(struct ipcbench_vtab *vtab) {
    //make_realtime(99);
    usleep(0.25e6);
    if( vtab->init_send ) vtab->init_send();

    while(1) {
        struct timespec ts = get_ticks();
        clock_gettime( CLOCK_MONOTONIC, &ts );
        vtab->send(&ts);
        usleep(1e3);
    }

    if( vtab->destroy_send ) vtab->destroy_send();
}

static void recv(struct ipcbench_vtab *vtab) {
    //make_realtime(99);
    if( vtab->init_recv ) vtab->init_recv();

    /* warm up */
    //for( size_t i = 0; i < 10; i ++ ) {
        //struct timespec ts;
        //vtab->recv(&ts );
    //}

    while(1) {
        struct timespec ts;
        vtab->recv(&ts );
        struct timespec now = get_ticks();
        printf("delta: %lf us\n", ticks_delta( ts, now ) * 1e6);
    }

    if( vtab->destroy_recv ) vtab->destroy_recv();
}

int main( int argc, char **argv ) {
    (void)argc;
    (void)argv;

    /* Parse args */
    const char *type = "ach";
    int c;
    while( (c = getopt( argc, argv, "v?V")) != -1 ) {
        switch(c) {
        case 'V':   /* version     */
            puts("ipcbench 0.0");
            exit(EXIT_SUCCESS);
        case '?':   /* version     */
            puts("Usage: ipcbench [OPTION....] TYPE\n"
                 "Benchmark IPC\n"
                 "\n"
                 "Options:\n"
                 "  -V                Version\n"
                 "  -?                Help\n"
                );
            exit(EXIT_SUCCESS);
        default:
            type = optarg;
        }
    }
    while( optind < argc ) {
        type = argv[optind++];
    }


    fprintf(stderr, "type: %s\n", type );

    /* Lookup vtab */
    struct {
        const char *name;
        struct ipcbench_vtab *vtab;
    } sym_vtabs[] = {
        {"ach", &ipc_bench_vtab_ach},
        {"lcm", &ipc_bench_vtab_lcm},
        {"pipe", &ipc_bench_vtab_pipe},
        {"mq", &ipc_bench_vtab_mq},
        {"tcp", &ipc_bench_vtab_tcp},
        {"local", &ipc_bench_vtab_local},
        {"udp", &ipc_bench_vtab_udp},
        {"localdgram", &ipc_bench_vtab_local_dgram},
        {NULL, NULL},
    };
    struct ipcbench_vtab *vtab;
    {
        size_t i = 0;
        while( sym_vtabs[i].name && 0 != strcasecmp( type, sym_vtabs[i].name ) ) i++;
        if( NULL == sym_vtabs[i].name ) {
            fprintf(stderr, "Invalid type: %s\n", type );
            exit(EXIT_FAILURE);
        }
        vtab = sym_vtabs[i].vtab;
    }

    /* Make realtime */
    make_realtime(30);
    calibrate();
    fprintf(stderr, "overhead: %f us\n", overhead * 1e6);


    /* Execute */
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
