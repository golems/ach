/* -*- mode: C; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * Copyright (c) 2015, Rice University
 * All rights reserved.
 *
 * Author(s): Neil T. Dantam <ntd@rice.edu>
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
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <stdlib.h>
#include <errno.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <string.h>
#include <stdio.h>
#include <pthread.h>
#include <unistd.h>
#include <ctype.h>
#include <stdbool.h>
#include <math.h>

#include <string.h>
#include <inttypes.h>

#include "ach.h"


typedef double x_t[3]; /* state / feedback: time, position, velocity */
typedef double u_t[1]; /* input message: velocity */

/* channels */
ach_channel_t chan_control;
ach_channel_t chan_feedback;

double now() {
    struct timespec t;
    clock_gettime( ACH_DEFAULT_CLOCK, &t );
    return (double)(t.tv_sec) + (double)t.tv_nsec / 1e9 ;
}

/* simple integrator, x = dt * dx */
void robot(void) {
    enum ach_status r = ach_open(&chan_feedback, "feedback", NULL);
    if(ACH_OK != r) abort();
    r = ach_open(&chan_control, "control", NULL);
    if(ACH_OK != r) abort();
    x_t X = {now(),0,0};
    while(1) {
        u_t U;
        size_t fs;
        r = ach_get( &chan_control, U, sizeof(U), &fs, NULL, ACH_O_WAIT|ACH_O_LAST );
        if( ach_status_match(r, ACH_MASK_OK | ACH_MASK_MISSED_FRAME) && sizeof(U) == fs ) {
            double tm = now();
            X[2] = U[0];               /*  dx = u       */
            X[1] = (tm - X[0]) * X[2]; /*  x = dt * dx  */
            X[0] = tm;
            ach_put(&chan_feedback, X, sizeof(X));
        } else abort();
    }
    exit(0);
}

/* print samples periodically */
void periodic_logger(void) {
    enum ach_status r = ach_open(&chan_feedback, "feedback", NULL);
    if(ACH_OK != r) abort();
    while(1) {
        x_t X;
        size_t fs;
        r = ach_get( &chan_feedback, X, sizeof(X), &fs, NULL, ACH_O_WAIT|ACH_O_LAST );
        if( ach_status_match(r, ACH_MASK_OK | ACH_MASK_MISSED_FRAME) && sizeof(X) == fs ) {
            printf("%f\t%f\t%f\n", X[0], X[1], X[2]);
            usleep((int) (1e6 * 0.1)); /* 10 Hertz */
        } else abort();
    }
    exit(0);
}

/* log all samples to a file */
void full_logger(void) {
    enum ach_status r = ach_open(&chan_feedback, "feedback", NULL);
    if(ACH_OK != r) abort();
    FILE *fp = fopen("ach-example.dat", "w");
    if(NULL == fp) abort();
    while(1) {
        x_t X;
        size_t fs;
        r = ach_get( &chan_feedback, X, sizeof(X), &fs, NULL, ACH_O_WAIT );
        if( ach_status_match(r, ACH_MASK_OK | ACH_MASK_MISSED_FRAME) ) {
            fprintf(fp,"%f\t%f\t%f\n", X[0], X[1], X[2]);
        } else abort();
    }
    fclose(fp);
    exit(0);
}

/* sinusoidal input */
void controller(void) {
    int r = ach_open(&chan_control, "control", NULL);
    if(ACH_OK != r) abort();
    while(1){
        double tm = now();
        u_t U = {sin(tm)};
        ach_put(&chan_control, U, sizeof(U));
        usleep((int)(1e6 * 1e-3)); /* kilohertz */
    }
    exit(0);
}

int main(int argc, char **argv) {
    (void) argc; (void)argv;
    enum ach_status r;
    /* create channels */
    r = ach_unlink("control");               /* delete first */
    if( ! ach_status_match(r, ACH_MASK_OK|ACH_MASK_ENOENT) ) abort();
    r = ach_unlink("feedback");              /* delete first */
    if( ! ach_status_match(r, ACH_MASK_OK|ACH_MASK_ENOENT) ) abort();
    r = ach_create("control", 10ul, 256ul, NULL );
    if(ACH_OK != r) abort();
    r = ach_create("feedback", 10ul, 256ul, NULL );
    if(ACH_OK != r) abort();

    /* fork processes */
    int pid_ctrl = fork();
    if(pid_ctrl < 0) abort();
    if(!pid_ctrl) controller();

    int pid_bot = fork();
    if(pid_bot < 0) abort();
    if(!pid_bot) robot();

    int pid_periodic = fork();
    if(pid_periodic < 0) abort();
    if(!pid_periodic) periodic_logger();

    int pid_full = fork();
    if(pid_full < 0) abort();
    if(!pid_full) full_logger();

    /* wait for a signal */
    pause();
    return 0;
}
