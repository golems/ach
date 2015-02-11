/*
 * Copyright (c) 2015, Rice University.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *     * Redistributions of source code must retain the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials
 *       provided with the distribution.
 *     * Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products
 *       derived from this software without specific prior written
 *       permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdlib.h>
#include <pthread.h>
#include <inttypes.h>
#include <ach.h>

#include <stdio.h>
#include <poll.h>
#include <unistd.h>

int main(int argc, char **argv)
{
    (void)argc; (void)argv;
    const char *names[] = {"channel-0", "channel-1"};
    const size_t n = sizeof(names) / sizeof(names[0]);
    ach_channel_t channel[n];
    struct pollfd pfd[n];
    size_t i;

    for( i = 0; i < n; i ++ ) {
        /* Open Channel */
        enum ach_status r = ach_open( &channel[i], names[i], NULL );
        if( ACH_OK != r ) {
            fprintf(stderr, "could not open channel '%s': %s\n",
                    names[i], ach_result_to_string(r));
            exit(EXIT_FAILURE);
        }
        /* Get Channel File Descriptor */
        r = ach_channel_fd( &channel[i], &pfd[i].fd );
        if( ACH_OK != r ) {
            fprintf(stderr, "could not get file descriptor for channel '%s': %s\n",
                    names[i], ach_result_to_string(r));
            exit(EXIT_FAILURE);
        }
        /* Set events to poll for */
        pfd[i].events = POLLIN;
    }

    /* read forever */
    for(;;) {
        /* poll for new messages */
        int r_poll = poll( pfd, n, -1 );
        if( r_poll < 0 ) {
            perror("poll");
            exit(EXIT_FAILURE);
        }
        /* find channels with new data */
        for( i = 0; i < n && r_poll > 0; i++ ) {
            if( (pfd[i].revents & POLLIN) ) {
                /* There's new data on this channel */
                char buf[512];
                size_t frame_size;
                enum ach_status r = ach_get( &channel[i], buf, sizeof(buf), &frame_size,
                                             NULL, ACH_O_NONBLOCK | ACH_O_FIRST );
                switch(r) {
                case ACH_OK:
                case ACH_MISSED_FRAME: {
                    /* this example just writes it to stdout */
                    ssize_t wr = write( STDOUT_FILENO, buf, frame_size );
                    if( wr < 0 ) {
                        perror("write");
                        exit(EXIT_FAILURE);
                    }
                    break;
                }
                default:
                    fprintf( stderr, "Error getting data from '%s': %s\n",
                             names[i], ach_result_to_string(r));
                    exit(EXIT_FAILURE);
                }
                r_poll--;
            }
        }
    }
    return 0;
}
