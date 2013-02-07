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

libach = CDLL("libach.so")


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


class ach_error(Exception):
    def __init__(self,value):
        self.value=value
    def __str__(self):
        return libach.ach_result_to_string(self.value)

def _ach_try(r):
    if 0 != r:
        raise ach_error(r)

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

    def put_buffer(self, buf, size):
        assert(self.is_open)
        _ach_try( libach.ach_put( self.pointer, buf, size ) )

    def put_string(self, string):
        self.put_buffer( c_char_p(string), len(string) )
