/*
 * Header file for ach_ipc kernel driver
 *
 * Copyright (C) 2013, Prevas A/S
 * Copyright (C) 2015, Rice University
 *
 * Authors: Kim Boendergaard Poulsen <kibo@prevas.dk>
 *          Neil T. Dantam <ntd@rice.edu>
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
 *   * Neither the name of Rice University nor the names of its
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
 *
 */

#ifndef ACH_KLINUX_H
#define ACH_KLINUX_H

#include "ach/generic.h"

/* The struct controlling channel devices a.k.a. /dev/ach-<channelname> */
struct ach_ch_device {
	struct ach_ch_device *next;
	int minor;
	char *name;
	struct mutex lock;	/* Protects open_files */
	struct cdev cdev;
	int open_files;
	struct ach_header *ach_data;	/* Has own lock to protect data */
};

/* The struct controlling the individual channel device file handles */
struct ach_ch_file {
	struct ach_ch_device *dev;
	struct ach_header *shm;	/* equals dev->ach_data - so not really necessary
				 * but we'll access ach_header using this pointer
				 */
	struct ach_ch_mode mode;

	// Stuff from userspace ach.h: ach_channel_t
	uint64_t seq_num;
		    /**< last sequence number read */
	size_t next_index;
		     /**< next index entry to try get from */
	unsigned int cancel;
};

typedef struct ach_ch_file ach_channel_t;

#endif

/* Local Variables:    */
/* mode: C             */
/* c-basic-offset: 8   */
/* indent-tabs-mode: t */
/* End:                */
