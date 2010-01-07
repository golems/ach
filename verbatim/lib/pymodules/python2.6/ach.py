## Copyright (c) 2010, Georgia Tech Research Corporation
## All rights reserved.
##
## Redistribution and use in source and binary forms, with or without
## modification, are permitted provided that the following conditions
## are met:
##
## * Redistributions of source code must retain the above copyright
##   notice, this list of conditions and the following disclaimer.
##
## * Redistributions in binary form must reproduce the above copyright
##   notice, this list of conditions and the following disclaimer in
##   the documentation and/or other materials provided with the
##   distribution.
##
## * Neither the name of the copyright holder(s) nor the names of its
##   contributors may be used to endorse or promote products derived
##   from this software without specific prior written permission.
##
## THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
## "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
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


## Author: Neil T. Dantam

import socket

class Channel:
    def __init__(self, name, mode, host="localhost", port=8075):
        assert "publish" == mode or "subscribe" == mode
        # Connect
        self.sock = socket.socket( socket.AF_INET, socket.SOCK_STREAM )
        self.sock.connect( (host, port) )
        # Headers
        self.mode = mode
        self.write_header( "mode", mode )
        self.write_header( "channel", name )
        if "subscribe" == mode:
            self.write_header( "synchronous", "hotdog" )
        self.end_headers()


    def write_header( self, label, value ):
        self.sock.sendall( "%s: %s\n" % (label, value) )
    def end_headers( self ):
        self.sock.sendall( "\n" )

    def publish( self, data ):
        assert "publish" == self.mode
        self.sock.sendall( "size" +
                           struct.pack('!I', len(data)) +
                           "data" +
                           data )
    def recv( self ):
        sizebuf = self.sock.recv( 12 )
        size_tag = sizebuf[0:4]
        size_data = sizebuf[4:8]
        data_tag = sizebuf[8:12]
        if "size" != size_tag or "data" != data_tag:
            raise Exception( "Invalid ach tags -- size: %s and data: %s" % (size_tag, data_tag) )
        size = struct.unpack('!I', size_data)[0]
        return self.sock.recv( size )

    def next( self ):
        assert "subscribe" == self.mode
        self.sock.sendall( "next" )
        return self.recv()

    def last( self ):
        assert "subscribe" == self.mode
        self.sock.sendall( "last" )
        return self.recv()

    def close( self ):
        self.sock.close()
