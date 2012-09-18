/* -*- mode: C; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2008-2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Neil T. Dantam <ntd@gatech.edu>
 * Georgia Tech Humanoid Robotics Lab
 * Under Direction of Prof. Mike Stilman <mstilman@cc.gatech.edu>
 *
 *
 * This file is provided under the following "BSD-style" License:
 *
 *
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 *
 */

/** \file achpipe-bin.c
 *  \author Neil T. Dantam
 *
 *  \bug does not fail gracefully when the channel can't be opened
 * \todo Extend protocol so that it only sends frames when requested
 */


/** \page pipe Pipe Protocol
 *
 * \section frame-format Frame Format
 *
 * <tt>
 * -------------------------------\n
 * | "achpipe" | uint64  | $DATA |\n
 * -------------------------------\n
 * </tt>
 *
 *
 * Ach frames are sent across the pipe as the 8 ascii bytes "achpipe"
 * (null terminated), a little-endian 64-bit integer indicating the
 * size in bytes of the frame, and the proper number of data bytes.
 *
 * This format is defined by the ach_pipe_frame_t struct.
 *
 * \section simple-mode Simple Mode
 *
 * A subscribing achpipe process will push frames across the pipe as
 * soon as they appear in the channel.  A publishing achpipe process
 * will read a frame from the pipe and the put it on the channel.
 *
 * \section sync-mode Synchronous Mode
 *
 * Subscribing achpipe processes can operate in synchronous mode.  In
 * this case, the process will read a command from the pipe and then
 * perform an action based on that command.  Current commands are:
 *
 *
 * <ul>
 * <li> \c next Send the next frame from channel. </li>
 * <li> \c last Send the last frame from channel.  </li>
 * </ul>
 *
 * These commands are sent as four ascii bytes, no '\\n' and no '\\0'.
 *
 * \section comparison-proper Comparison to Ach Proper
 *
 * Note that ach normally communicates using shared memory.  The pipe
 * protocol is only for intermachine communication or increasing
 * robustness if one does not want a process to touch shared memory
 * directly.
 *
 *
 * \sa Todo List
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <pthread.h>
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include <unistd.h>
#include <stdarg.h>
#include <string.h>
#include <time.h>
#include <errno.h>
#include <inttypes.h>
#include "ach.h"




/** sleep for specified time */
static int _relsleep( struct timespec t ) {
    struct timespec rem;
    int r;
    do {
        r = nanosleep( &t, &rem );
        assert( 0 == r || EINTR == errno );
        t.tv_sec = rem.tv_sec;
        t.tv_nsec = rem.tv_nsec;
    } while( 0 != r ) ;
    return 0;
}






/* FIXME: It seems that the kernel buffers very small messages.  This
 * is really bad, because we want the data fast.  Should come up with
 * some way to force TCP to just send it.
 *
 * The answer: TCP_NODELAY
 *
 */



/* lets pick the number POSIX specifies for atomic reads/writes */
/** Initial size of ach frame buffer */
#define INIT_BUF_SIZE 512

/*
  #define HEADER_LINE_MAX 4096
  #define HEADER_LABEL_MAX 512
  #define HEADER_VALUE_MAX 4096
*/

/*#define DEBUGF(fmt, a... ) */
/*#define DEBUGF(fmt, a... ) */
/*fprintf(stderr, (fmt), ## a ) */

/** CLI option: channel name */
char opt_chan_name[ACH_CHAN_NAME_MAX + 2] = {0};
/** CLI option: remote channel name */
char opt_remote_chan_name[ACH_CHAN_NAME_MAX + 2] = {0};
/** CLI option: publish mode */
int opt_pub = 0;
/** CLI option: subscribe mode */
int opt_sub = 0;
/** CLI option: verbosity level */
int opt_verbosity = 0;
/** CLI option: send only most recent frames */
int opt_last = 0;
/** CLI option: synchronous mode */
int opt_sync = 0;
/** CLI option: frequency */
double opt_freq = 0;
/*
/// CLI option: read option headers
int opt_read_headers = 0;
/// CLI option: write option headers
int opt_write_headers = 0;
*/

/** got a signel yet? */
int sig_received = 0;

/** print stuff based on verbosity level */
void verbprintf( int level, const char fmt[], ... ) {
    va_list args;
    va_start( args, fmt );
    if( level <= opt_verbosity ) {
        fprintf(stderr, "achpipe: ");
        vfprintf( stderr, fmt, args );
    }
    va_end( args );
}

/** Check test, if false, print error and abort. */
void hard_assert(int test, const char fmt[], ... ) {
    if( !test ) {
        va_list args;
        va_start( args, fmt );
        fprintf(stderr, "ERR: ");
        vfprintf( stderr, fmt, args );
        va_end( args );
        putc( '\n', stderr );
        abort();
        exit(1);
    }
}

/* static void *xmalloc( size_t size ) { */
/*     void *p = malloc( size ); */
/*     if( NULL == p ) { */
/*         fprintf(stderr, "Couldn't allocate %"PRIuPTR" bytes\n", size ); */
/*         perror("malloc"); */
/*         abort(); */
/*     } */
/*     return p; */
/* } */
/*
  int read_header( char label[HEADER_LABEL_MAX],
  char value[HEADER_VALUE_MAX] ) {
  // read line
  char line[HEADER_LINE_MAX];
  size_t n_line = ach_read_line( STDIN_FILENO, line,
  HEADER_LINE_MAX );
  assert( '\0' == line[n_line] );
  hard_assert( '\n' == line[n_line-1],
  "Couldn't read header line: %s", line );
  size_t i = 0;
  // skip whitespace
  while( (' ' ==  line[i] || '\t' == line[i]) &&
  i < HEADER_LINE_MAX - 1 ) {
  i++;
  }
  assert( i < HEADER_LINE_MAX );
  // check for blank line
  if( '\n' == line[i] ) {
  assert( i + 1 == n_line );
  return 1;
  }
  // split label
  {
  size_t j = 0;
  while( line[i] != ':' &&
  j < HEADER_LABEL_MAX - 1 &&
  i < HEADER_LINE_MAX - 1) {
  label[j++] = line[i++];
  }
  hard_assert( i < HEADER_LABEL_MAX,
  "Label too long in header: %s", line);
  assert( ':' == line[i] );
  label[i++] = '\0';
  }
  // skip white space
  while( (' ' ==  line[i] || '\t' == line[i]) &&
  i < HEADER_LINE_MAX ) {
  i++;
  }
  // split value
  {
  size_t j = 0;
  while( line[i] != '\n' &&
  i < HEADER_LINE_MAX - 1 &&
  j < HEADER_VALUE_MAX - 1) {
  assert( '\0' != line[i] );
  value[j++] = line[i++];
  }
  assert( i < HEADER_LINE_MAX - 1  );
  assert( '\0' == line[i+1] );
  assert( i + 1 == n_line );
  hard_assert( j < HEADER_VALUE_MAX,
  "Value too long in header: %s", line);
  value[j] = '\0';
  }

  return 0;
  }

  void parse_headers() {
  char label[HEADER_LABEL_MAX];
  char value[HEADER_VALUE_MAX];

  while( 0 == read_header( label, value ) )  {
  if( 0 == strcasecmp( "mode", label ) ) {
  if( 0 == strcasecmp( "publish", value ) ) {
  opt_pub = 1;
  } else if ( 0 == strcasecmp( "subscribe", value ) ) {
  opt_sub = 1;
  } else {
  hard_assert( 0, "Invalid mode header: %s", value );
  }
  } else if( 0 == strcasecmp( "channel", label ) ) {
  hard_assert( strlen( label ) < ACH_CHAN_NAME_MAX-1,
  "Channel name too long" );
  strcpy( opt_chan_name, value );
  } else {
  hard_assert( 0, "Invalid header label: %s", label );
  }
  }
  }

  void write_header( char *label, char *value ) {
  char line[HEADER_LINE_MAX];
  size_t n_label = strnlen( label, HEADER_LINE_MAX );
  size_t n_value = strnlen( value, HEADER_LINE_MAX );
  hard_assert( n_label + n_value + 5 < HEADER_LINE_MAX,
  "trying to write too long header\n" );
  strcpy( line, label );
  strcat( line, ": " );
  strcat( line, value );
  strcat( line, "\n" );
  int n = strlen( line );
  assert( n + 1 < HEADER_LINE_MAX );
  int r = ach_stream_write_fill( STDOUT_FILENO, line, n );
  hard_assert( n == r,
  "Failed to write header, n = %d, r = %d\n", n, r );
  }

  void write_headers() {
  write_header( "mode", opt_pub ? "subscribe" : "publish" );
  write_header( "channel", opt_remote_chan_name[0] ? opt_remote_chan_name : opt_chan_name );
  int r = ach_stream_write_fill( STDOUT_FILENO, "\n", 1 );
  hard_assert( 1 == r, "Couldn't write header newline\n" );
  }
*/

/** publishing loop */
void publish( FILE *fin, char *chan_name )  {
    verbprintf(1, "Publishing()\n");
    //assert(STDIN_FILENO == fd );
    ach_channel_t chan;

    { /* open channel */
        ach_status_t r;
        r = ach_open( &chan, chan_name, NULL );
        hard_assert(ACH_OK == r,
                    "Failed to open channel %s for publish: %s\n",
                    chan_name, ach_result_to_string(r) );
    }

    { /* publish loop */
        uint64_t max = INIT_BUF_SIZE;
        ach_pipe_frame_t *frame = ach_pipe_alloc( max );
        while( ! sig_received ) {
            /* get size */
            size_t s = fread( frame, 1, 16, fin );
            verbprintf( 2, "Read %d bytes\n", s );
            if( 16 != s ) break;
            if( memcmp("achpipe", frame->magic, 8) ) break;
            uint64_t cnt = ach_pipe_get_size( frame );
            // FIXME: sanity check that cnt is not something outrageous
            /* make sure buf can hold it */
            if( (size_t)cnt > max ) {
                max = cnt;
                free( frame );
                frame = ach_pipe_alloc( max );
            }
            /* get data */
            s = fread( frame->data, 1, (size_t)cnt, fin );
            if( cnt != s ) break;
            /* put data */
            ach_status_t r = ach_put( &chan, frame->data, cnt );
            hard_assert( r == ACH_OK, "Invalid ach put %s\n",
                         ach_result_to_string( r ) );
        }
        free(frame);

    }
    ach_close( &chan );
}


/** subscribing loop */
void subscribe( FILE *fin, FILE *fout, char *chan_name ) {
    verbprintf(1, "Subscribing()\n");
    verbprintf(1, "Synchronous: %s\n", opt_sync ? "yes" : "no");
    /* get channel */
    ach_channel_t chan;
    {
        ach_status_t r = ach_open( &chan, chan_name, NULL );
        hard_assert( ACH_OK == r,
                     "Failed to open channel %s for subscribe: %s\n",
                     chan_name, ach_result_to_string(r) );
        r = ach_flush( &chan );
        hard_assert( ACH_OK == r,
                     "Failed to flush channel %s for on: %s\n",
                     chan_name, ach_result_to_string(r) );
    }
    /* frame buffer */
    size_t max = INIT_BUF_SIZE;
    ach_pipe_frame_t *frame = ach_pipe_alloc( max );
    int t0 = 1;


    struct timespec period = {0,0};
    int is_freq = 0;
    if(opt_freq > 0) {
        double p = 1.0 / opt_freq;
        period.tv_sec = (time_t)p;
        period.tv_nsec = (long) ((p - (double)period.tv_sec)*1e9);
        is_freq = 1;
    }

    /* read loop */
    while( ! sig_received ) {
        char cmd[4] = {0};
        if( opt_sync ) {
            /* wait for the pull command */
            size_t rc = fread( cmd, 1, 4, fin);
            hard_assert(4 == rc, "Invalid command read: %d\n", rc );
            verbprintf(2, "Command %s\n", cmd );
        }
        /* read the data */
        int got_frame = 0;
        do {
            size_t frame_size = 0;
            ach_status_t r = ACH_BUG;
            if( opt_sync ) {
                /* parse command */
                if ( 0 == memcmp("next", cmd, 4) ) {
                    r = ach_get(&chan, frame->data, max, &frame_size,  NULL,
                                ACH_O_WAIT );
                }else if ( 0 == memcmp("last", cmd, 4) ){
                    r = ach_get(&chan, frame->data, max, &frame_size,  NULL,
                                ACH_O_WAIT | ACH_O_LAST );
                } else if ( 0 == memcmp("poll", cmd, 4) ) {
                    r = ach_get( &chan, frame->data, max, &frame_size, NULL,
                                 ACH_O_COPY | ACH_O_LAST );
                } else {
                    hard_assert(0, "Invalid command: %s\n", cmd );
                }
            } else {
                /* push the data */
                r = (opt_last || is_freq) ?
                    ach_get( &chan, frame->data, max, &frame_size,  NULL, ACH_O_WAIT | ACH_O_LAST ) :
                    ach_get( &chan, frame->data, max, &frame_size,  NULL, ACH_O_WAIT ) ;
            }
            /* check return code */
            if( ACH_OVERFLOW == r ) {
                /* enlarge buffer and retry on overflow */
                assert(frame_size > max );
                max = frame_size;
                free(frame);
                frame = ach_pipe_alloc( max );
            } else if (ACH_OK == r || ACH_MISSED_FRAME == r || t0 ) {
                got_frame = 1;
                ach_pipe_set_size( frame, frame_size );
                verbprintf(2, "Got ach frame %d\n", frame_size );
            }else {
                /* abort on other errors */
                hard_assert( 0, "sub: ach_error: %s\n",
                             ach_result_to_string(r) );
                assert(0);
            }
        }while( !got_frame );


        /* stream send */
        {
            size_t size = sizeof(ach_pipe_frame_t) - 1 + ach_pipe_get_size(frame);
            size_t r = fwrite( frame, 1, size, fout );
            if( r != size ) {
                break;
            }
            if( fflush( fout ) ) break;
            if( opt_sync ) {
                fsync( fileno(fout) ); /* fails w/ sbcl, and maybe that's ok */
            }
            verbprintf( 2, "Printed output\n");
        }
        t0 = 0;
        /* maybe sleep */
        if( is_freq ) {
            assert( !opt_sync );
            _relsleep(period);
        }
    }
    free(frame);
    ach_close( &chan );
}


static void sighandler(int sig, siginfo_t *siginfo, void *context) {
    (void) context;
    verbprintf (1,
                "Received Signal: %d, Sending PID: %ld, UID: %ld\n",
                sig, (long)siginfo->si_pid, (long)siginfo->si_uid);
    sig_received = 1;
}

/** setup the signal handler */
void sighandler_install() {
    struct sigaction act;
    memset(&act, 0, sizeof(act));

    act.sa_sigaction = &sighandler;

    /* The SA_SIGINFO flag tells sigaction() to use the sa_sigaction field,
       not sa_handler. */
    act.sa_flags = SA_SIGINFO;

    if (sigaction(SIGTERM, &act, NULL) < 0) {
        perror ("sigaction");
        fprintf(stderr, "Couldn't install handler\n");
        abort();
    }

    if (sigaction(SIGINT, &act, NULL) < 0) {
        perror ("sigaction");
        fprintf(stderr, "Couldn't install handler\n");
        abort();
    }
}



/** main */
int main( int argc, char **argv ) {
    int c;
    while( (c = getopt( argc, argv, "p:s:z:vlcf:h?")) != -1 ) {
        switch(c) {
        case 'p':
            opt_pub = 1;
            hard_assert( strlen( optarg ) < ACH_CHAN_NAME_MAX-1,
                         "Channel name argument to long" );
            strncpy( opt_chan_name, optarg, ACH_CHAN_NAME_MAX );
            break;
        case 's':
            opt_sub = 1;
            hard_assert( strlen( optarg ) < ACH_CHAN_NAME_MAX-1,
                         "Channel name argument to long" );
            strncpy( opt_chan_name, optarg, ACH_CHAN_NAME_MAX );
            break;
        case 'z':
            hard_assert( strlen( optarg ) < ACH_CHAN_NAME_MAX-1,
                         "Channel name argument to long" );
            strncpy( opt_remote_chan_name, optarg, ACH_CHAN_NAME_MAX );
        case 'v':
            opt_verbosity ++;
            break;
        case 'l':
            opt_last = 1;
            break;
        case 'c':
            opt_sync = 1;
            break;
        case 'f':
            opt_freq = atof(optarg);
            break;
        default:
            puts( "Usage: achpipe.bin [OPTION...]\n"
                  "Translate between ach channels and streams"
                  "\n"
                  "  -p CHANNEL-NAME,     Publish stream to channel\n"
                  "  -s CHANNEL-NAME,     Subscribe from channel, print to stream\n"
                  "  -z CHANNEL-NAME,     Set name of remote channel\n"
                  "  -l CHANNEL-NAME,     Get latest messages\n"
                  "  -c,                  Synchronous mode\n"
                  "  -f FREQUENCY,        Output to stream at FREQUENCY\n"
                  "  -o OCTAL,            Mode for created channel\n"
                  "  -v,                  Be verbose\n" );
            exit(EXIT_SUCCESS);
        }
    }

    /*
      hard_assert( ! ( opt_write_headers &&  opt_read_headers ),
      "can't read and write headers\n" );

      if( opt_read_headers ) {
      parse_headers();
      }else if( opt_write_headers ) {
      write_headers();
      }
    */

    /* validate arguments */
    hard_assert( 0 < strlen( opt_chan_name ),
                 "must specify channel\n" );
    hard_assert( ! ( opt_pub &&  opt_sub ),
                 "must specify publish or subscribe mode\n" );
    hard_assert( opt_pub || opt_sub ,
                 "must specify publish xor subscribe mode\n" ) ;
    hard_assert( !(0.0 > opt_freq || 0.0 < opt_freq)  || (!opt_sync && opt_sub),
                 "frequency only valid on async subscribe mode\n" );
    hard_assert( opt_freq >= 0, "frequency must be positive\n" );

    /* maybe print */
    verbprintf( 1, "Channel: %s\n", opt_chan_name );
    verbprintf( 1, "Publish: %s\n",  opt_pub ? "yes" : "no" );
    verbprintf( 1, "Subscribe: %s\n", opt_sub ? "yes" : "no" );

    /* install sighandler */
    sighandler_install();
    /* run */
    if (opt_pub) {
        publish( stdin, opt_chan_name );
    } else if (opt_sub) {
        subscribe( stdin, stdout, opt_chan_name );
    } else {
        assert(0);
    }
    return 0;
}
