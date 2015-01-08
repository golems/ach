#ifndef ACH_KLINUX_GENERIC_H
#define ACH_KLINUX_GENERIC_H
/*
 * Header file for ach_ipc kernel driver
 *
 * Copyright (C) 2013, Prevas A/S
 *
 * Authors: Kim Boendergaard Poulsen <kibo@prevas.dk>
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
 *
 */

#define ACH_NAME                    "ach-ipc"
#define ACH_IOCTL                   'Q'
#define ACH_MAX_DEVICES             512

#define ACH_CTRL_CREATE_CH              _IOW(ACH_IOCTL, 1, struct ach_ctrl_create_ch)
#define ACH_CTRL_UNLINK_CH              _IOW(ACH_IOCTL, 2, struct ach_ctrl_unlink_ch)

#define ACH_CH_CANCEL_UNSAFE            0x01

#define ACH_CH_SET_MODE                 _IOW(ACH_IOCTL, 3, struct ach_ch_mode)
#define ACH_CH_GET_MODE                 _IOR(ACH_IOCTL, 4, struct ach_ch_mode)
#define ACH_CH_GET_STATUS               _IOR(ACH_IOCTL, 5, struct ach_ch_status)
#define ACH_CH_FLUSH                    _IOW(ACH_IOCTL, 6, unsigned int)
#define ACH_CH_CANCEL                   _IOW(ACH_IOCTL, 7, unsigned int)

struct ach_ctrl_create_ch {
	size_t frame_cnt;
	size_t frame_size;
	char name[ACH_CHAN_NAME_MAX + 1];
};

struct ach_ctrl_unlink_ch {
	char name[ACH_CHAN_NAME_MAX + 1];
};

struct ach_ch_mode {
	unsigned int mode;
	struct timespec reltime;	/* Relative time - notice ach normally runs abstime */
};

struct ach_ch_status {
	unsigned int mode;
	ssize_t size;		/* Size of queue */
	ssize_t count;		/* Messages in queue */
	ssize_t new;		/* Unread messages in queue */
	unsigned long last_seq;	/* Last sequence in queue */
	unsigned long last_seq_read;	/* Sequence of last read message */
};

#endif

/* Local Variables:    */
/* mode: C             */
/* c-basic-offset: 8   */
/* indent-tabs-mode: t */
/* End:                */
