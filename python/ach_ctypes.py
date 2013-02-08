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

from ctypes import *


CTYPES Junk
libach = CDLL("libach.so")

## Prototypes ##
libach.ach_channel_alloc.argtypes = []
libach.restype = c_void_p

libach.ach_channel_free.argtypes = [c_void_p]
libach.ach_channel_free.restype = None

libach.ach_result_to_string.argtypes = [c_int]
libach.ach_result_to_string.restype = c_char_p

libach.ach_open.argtypes = [c_void_p, c_char_p, c_void_p]
libach.ach_open.restype = c_int

libach.ach_close.argtypes = [c_void_p]
libach.ach_close.restype = c_int

libach.ach_put.argtypes = [c_void_p, c_void_p, c_size_t]
libach.ach_put.restype = c_int

libach.ach_get.argtypes = [c_void_p, c_void_p, c_size_t, POINTER(c_size_t),
                           c_void_p, c_int]
libach.ach_get.restype = c_int


## Extract Constants from shared library ##
ACH_OK             = c_int.in_dll( libach, "ach_ok" ).value
ACH_OVERFLOW       = c_int.in_dll( libach, "ach_overflow" ).value
ACH_INVALID_NAME   = c_int.in_dll( libach, "ach_invalid_name" ).value
ACH_BAD_SHM_FILE   = c_int.in_dll( libach, "ach_bad_shm_file" ).value
ACH_FAILED_SYSCALL = c_int.in_dll( libach, "ach_failed_syscall" ).value
ACH_STALE_FRAMES   = c_int.in_dll( libach, "ach_stale_frames" ).value
ACH_MISSED_FRAME   = c_int.in_dll( libach, "ach_missed_frame" ).value
ACH_TIMEOUT        = c_int.in_dll( libach, "ach_timeout" ).value
ACH_EEXIST         = c_int.in_dll( libach, "ach_eexist" ).value
ACH_ENOENT         = c_int.in_dll( libach, "ach_enoent" ).value
ACH_CLOSED         = c_int.in_dll( libach, "ach_closed" ).value
ACH_BUG            = c_int.in_dll( libach, "ach_bug" ).value
ACH_EINVAL         = c_int.in_dll( libach, "ach_einval" ).value
ACH_CORRUPT        = c_int.in_dll( libach, "ach_corrupt" ).value
ACH_BAD_HEADER     = c_int.in_dll( libach, "ach_bad_header" ).value
ACH_EACCES         = c_int.in_dll( libach, "ach_eacces" ).value
ACH_O_WAIT         = c_int.in_dll( libach, "ach_o_wait" ).value
ACH_O_LAST         = c_int.in_dll( libach, "ach_o_last" ).value

## Execptions ##
class ach_error(Exception):
    def __init__(self,value):
        self.value=value
    def __str__(self):
        return libach.ach_result_to_string(self.value)

def _ach_try(r):
    if 0 != r:
        raise ach_error(r)

import ach_py

## Channel Object ##
class channel:
    ''' An Ach channel'''
    def __init__(self):
        self.pointer = libach.ach_channel_alloc()
        self.is_open = False

    def __del__(self):
        # libach becomes undefined when python is exiting
        if libach:
            if( self.is_open ):
                self.close()
            libach.ach_channel_free( self.pointer )

    def open(self, name):
        assert( not self.is_open )
        print "opening"
        _ach_try( libach.ach_open( self.pointer, name, None ) )
        self.is_open = True

    def close(self):
        assert(self.is_open)
        print "closing"
        _ach_try( libach.ach_close( self.pointer ) )
        self.is_open = False

    # raw put/get
    def put_pointer(self, buf, size):
        assert(self.is_open)
        _ach_try( libach.ach_put( self.pointer, buf, size ) )

    def get_pointer(self, buf, size, wait=False, last=False):
        opts = 0
        if wait:
            opts = opts | libach.ach_o_wait
        if last:
            opts = opts | libach.ach_o_last
        frame_size = c_size_t()
        _ach_try( libach.ach_get( self.pointer, buf, size,
                                  byref(frame_size), None, opts ) )
        return frame_size.value

    # typed put/get
    def put_string(self, string):
        self.put_pointer( c_char_p(string), len(string) )
    def put_ctype(self, obj):
        self.put_pointer( pointer(obj), sizeof(obj) )
    def put_bytearray(self, obj):
        self.put_ctype( (c_byte * len(obj)).from_buffer(obj) )


    def get_ctype(self, obj, wait=False, last=False):
        return self.get_pointer( pointer(obj), sizeof(obj), wait=wait, last=last )
    def get_bytearray(self, obj, wait=False, last=False):
        fs = self.get_ctype( (c_byte*len(obj)).from_buffer(obj), wait, last )
        return obj


