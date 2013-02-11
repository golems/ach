#!/usr/bin/env python

## Copyright (c) 2013, Georgia Tech Research Corporation
## All rights reserved.
##
## Author(s): Neil T. Dantam <ntd@gatech.edu>
## Georgia Tech Humanoid Robotics Lab
## Under Direction of Prof. Mike Stilman <mstilman@cc.gatech.edu>
##
## Redistribution and use in source and binary forms, with or without
## modification, are permitted provided that the following conditions
## are met:
##
##     * Redistributions of source code must retain the above
##       copyright notice, this list of conditions and the following
##       disclaimer.
##     * Redistributions in binary form must reproduce the above
##       copyright notice, this list of conditions and the following
##       disclaimer in the documentation and/or other materials
##       provided with the distribution.
##
## THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
## ''AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
## LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
## FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
## COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
## INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
## (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
## SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
## HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
## STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
## ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
## OF THE POSSIBILITY OF SUCH DAMAGE.

## FILE: ach.py
## DESC: python module to access ach

import ach_py

from ach_py import \
    ACH_OK,\
    ACH_OVERFLOW,\
    ACH_INVALID_NAME,\
    ACH_BAD_SHM_FILE,\
    ACH_FAILED_SYSCALL,\
    ACH_STALE_FRAMES,\
    ACH_MISSED_FRAME,\
    ACH_TIMEOUT,\
    ACH_EEXIST,\
    ACH_ENOENT,\
    ACH_CLOSED,\
    ACH_BUG,\
    ACH_EINVAL,\
    ACH_CORRUPT,\
    ACH_BAD_HEADER,\
    ACH_EACCES,\
    ACH_O_WAIT,\
    ACH_O_LAST, \
    AchException

class Channel:
    """ An Ach channel."""

    def __init__(self, name = None):
        '''Create a channel.

        name -- if provided, the name of the channel to open
        '''
        self.pointer = None
        if name:
            self.open(name)

    def __del__(self):
        '''Close the channel.'''
        if ach_py and self.pointer:
            self.close()

    def open(self, name):
        '''Open channel with the given name.'''
        assert( not self.pointer )
        self.pointer = ach_py.open_channel(name)

    def close(self):
        '''Close the channel.'''
        assert(self.pointer)
        ach_py.close_channel(self.pointer)
        self.pointer = None

    def put(self, buf):
        '''Put a buffer into the channel.

        buf -- an object providing the buffer interface'''
        assert(self.pointer)
        ach_py.put_buf( self.pointer, buf )

    def get(self, buf, wait=False, last=False):
        '''Get a message from channel and write to buffer.

        buf -- an object providing the buffer interface
        wait -- wait until new message posts
        last -- get most recent message

        returns a tuple of (ach_status, frame_size)'''
        assert(self.pointer)
        return ach_py.get_buf( self.pointer, buf, wait, last )

    def result_string( self, status ):
        return ach_py.result_string(status)
