/*
 * Copyright (c) 2013, Georgia Tech Research Corporation
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

package org.golems.ach;

/** An Ach channel.
 */
class Channel
{
    /** Create a channel with default frame size
     */
    public Channel() {}

    /** Initialize a channel with a given nominal frame_size.
     *
     * The frame size is used to preallocate buffers when receiving a
     * message.  A mismatch between the value given here and the size
     * of actual messages may result in extra buffer allocation.
     */
    public Channel(long frame_size) {
        this.frame_size = frame_size;
    }

    /** Open channel with the give channel_name.
     */
    public int open( String channel_name ) {
        long[] ptr = new long[1];
        int r = Lib.open(channel_name, ptr);
        this.chan_ptr = ptr[0];
        return r;
    }

    /** Close the channel. */
    public int close() {
        if( chan_ptr != 0 ) {
            int r = Lib.close( this.chan_ptr );
            chan_ptr = 0;
            return r;
        } else return Lib.OK;
    }


    /** Close the channel when garbage collected.
     *
     * This will free() C-heap data allocated by open().
     */
    protected void finalize() throws Throwable {
        try {
            close();
        } finally {
            super.finalize();
        }
    }

    /** Discard old messages in the channel. */
    public int flush() {
        return Lib.flush( this.chan_ptr );
    }

    /** Discard old messages in the channel. */
    public void flush(int status_mask) throws Status.AchException {
        int r = flush();
        Status.maybe_throw( r, status_mask );
    }

    /** Write buf to channel. */
    public int put( byte[] buf ) {
        return Lib.put( this.chan_ptr, buf );
    }

    /** Write buf to channel. */
    public void put( byte[] buf, int status_mask ) throws Exception {
        int r = Lib.put( this.chan_ptr, buf );
        Status.maybe_throw( r, status_mask );
    }

    /** Write string to channel. */
    public int put( String str ) {
        try {
            return Lib.put( this.chan_ptr, str.getBytes("UTF-8") );
        } catch( java.io.UnsupportedEncodingException e) {
            return Lib.BUG;
        }
    }

    /** Get buffer from channel. */
    public int get( byte[] buf, long[] frame_size, int options ) {
        return Lib.get( this.chan_ptr, buf, frame_size, options );
    }

    /** Get buffer from channel. */
    public byte[] get( int options, int status_mask ) throws Status.AchException {
        byte[] buf;
        long[] frame_size = new long[1];
        frame_size[0] = this.frame_size; // default frame size
        int r;

        /* Loop till message fits in our buffer */
        do {
            buf = new byte[ (int)frame_size[0] ];
            r = get( buf, frame_size, options );
        } while( Lib.OVERFLOW == r );

        /* Check Status */
        Status.maybe_throw( r, status_mask );

        /* Maybe extract subarray */
        if( buf.length == frame_size[0] ) {
            return buf;
        } else {
            return java.util.Arrays.copyOfRange( buf, 0, (int)frame_size[0] );
        }
    }

    /** C pointer to the channel. */
    private long chan_ptr = 0;

    /** Nominal frame size */
    private long frame_size = 512;

}




