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


#include <time.h>
#include <stdint.h>
#include <fcntl.h>
#include <ach.h>
#include <errno.h>

#include <unistd.h>
#include <sys/epoll.h>

#include <stdio.h>


enum ach_status
ach_evhandle( struct ach_evhandler *handlers,
              size_t n,
              const struct timespec *period,
              enum ach_status (*periodic_handler)(void *context),
              void *periodic_context,
              int options )
{
    enum ach_status r;

    size_t n_kernel = 0;
    enum ach_map map[n];
    int efd = -1;

    _Bool periodic_input = options & ACH_EV_O_PERIODIC_INPUT;
    _Bool periodic_timeout = options & ACH_EV_O_PERIODIC_TIMEOUT;

    /* Count kernel channels */
    for( size_t i = 0; i < n; i ++ ) {
        if( ACH_OK != (r = ach_channel_mapping(handlers[i].channel, map+i)) )
            return r;
        if( ACH_MAP_KERNEL == map[i] ) n_kernel++;
    }

    /* TODO: support user channels via polling at the requested period */
    if (n_kernel != n ) return ACH_EINVAL;

    /* Initialize epoll */
    if( n_kernel ) {
        errno = 0;
        if( 0 > (efd = epoll_create1(EPOLL_CLOEXEC)) ) goto END;
        for( size_t i = 0; i < n; i ++ ) {
            if( ACH_MAP_KERNEL == map[i] ) {
                struct epoll_event event = {0};
                int fd;
                if( ACH_OK != (r = ach_channel_fd(handlers[i].channel, &fd))) goto END;
                event.events = EPOLLIN;
                event.data.ptr = handlers + i;
                if( epoll_ctl( efd, EPOLL_CTL_ADD, fd, &event ) ) {
                    r = ACH_FAILED_SYSCALL; goto END;
                }
            }
        }
    }

    /* Initialize timeouts */
    int period_ms;
    if( period ) {
        period_ms = (int)(period->tv_sec*1000 + period->tv_nsec/1000000 );
        /* TODO: is this reasonable?
         * Maybe we should just select() with microseconds
         */
        if( 0 == period_ms && 0 != period->tv_nsec ) {
            period_ms = 1;
        }
    } else {
        period_ms = -1;
    }
    int remaining_ms = period_ms;

    struct timespec now, then;
    if( period ) clock_gettime(ACH_DEFAULT_CLOCK, &then);

    /* Event loop */
    do {
        /* TODO: maybe wait for multiple events */
        struct epoll_event event;
        const int r_epoll = epoll_wait(efd, &event, 1, remaining_ms);
        if( r_epoll < 0 ) {
            r = ACH_FAILED_SYSCALL;
        } else if( r_epoll > 0 ) {
            /* got some input */
            if( event.events & EPOLL_CTL_ADD ) {
                struct ach_evhandler *handler =  (struct ach_evhandler*) event.data.ptr;
                r = handler->handler(handler->context, handler->channel);
            }
            /* periodic handler on input */
            if( ACH_OK == r && periodic_handler && periodic_input ) {
                r = periodic_handler(periodic_context);
            }
        }

        /* Update timeout */
        if( ACH_OK == r && period ) {
            clock_gettime(ACH_DEFAULT_CLOCK, &now);
            int elapsed_ms = (int)( (now.tv_sec - then.tv_sec)*1000 -
                                    (now.tv_nsec - then.tv_nsec) / 1000000 );
            if( elapsed_ms >= period_ms  ) {
                /* periodic handler on timeout */
                if( periodic_handler && periodic_timeout ) {
                    r = periodic_handler(periodic_context);
                }
                remaining_ms = period_ms;
                then = now;
            } else {
                remaining_ms = period_ms - elapsed_ms;
            }
        }
    } while( ACH_OK == r );

END:
    if( efd > 0 ) {
        if( errno ) {
            /* Keep the original errno */
            int tmp = errno;
            close(efd);
            errno = tmp;
        } else {
            close(efd);
        }
    }
    return r;
}
