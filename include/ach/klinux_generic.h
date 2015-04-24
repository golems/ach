#ifndef ACH_KLINUX_GENERIC_H
#define ACH_KLINUX_GENERIC_H
/*
 * Header file for ach_ipc kernel driver
 *
 * Copyright (C) 2013, Prevas A/S
 * Copyright (c) 2015, Atlas Copco Rock Drills AB
 *
 * Authors: Kim Boendergaard Poulsen <kibo@prevas.dk>
 *          Mattias <matjo75@gmail.com>
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

/** \file klinux_generic.h
 *
 *  \brief This file contains declarations for the kernel
 *         implementation needed by both user and kernel space.
 *
 *  \author Kim Boendergaard Poulsen
 */

/** Kernel device subsystem name for ach channel devices */
#define ACH_CH_SUBSYSTEM            "ach_ch"

/** ioctl type for ach control and channel devices */
#define ACH_IOCTL                   'Q'

/** ioctl request code to create a channel */
#define ACH_CTRL_CREATE_CH              _IOW(ACH_IOCTL, 1, struct ach_ctrl_create_ch)

/** ioctl request code to unlink a channel */
#define ACH_CTRL_UNLINK_CH              _IOW(ACH_IOCTL, 2, struct ach_ctrl_unlink_ch)

#define ACH_CH_CANCEL_UNSAFE            0x01

/** ioctl request set channel mode options */
#define ACH_CH_SET_MODE                 _IOW(ACH_IOCTL, 3, struct achk_opt)
/** ioctl request get channel mode options */
#define ACH_CH_GET_MODE                 _IOR(ACH_IOCTL, 4, struct achk_opt)
/** ioctl request get channel status */
#define ACH_CH_GET_STATUS               _IOR(ACH_IOCTL, 5, struct ach_ch_status)
/** ioctl request flush channel */
#define ACH_CH_FLUSH                    _IOW(ACH_IOCTL, 6, unsigned int)
/** ioctl request cancel channel reads */
#define ACH_CH_CANCEL                   _IOW(ACH_IOCTL, 7, unsigned int)
/** ioctl request cancel channel reads */
#define ACH_CH_GET_OPTIONS              _IOW(ACH_IOCTL, 8, struct ach_ch_options)

#ifdef CONFIG_COMPAT

#include <linux/types.h>

/** compat ioctl requests */
#define ACH_CTRL_CREATE_CH_COMPAT       _IOW(ACH_IOCTL, 1, struct ach_ctrl_create_ch_32)
#define ACH_CH_SET_MODE_COMPAT          _IOW(ACH_IOCTL, 3, struct achk_opt_32)
#define ACH_CH_GET_MODE_COMPAT          _IOR(ACH_IOCTL, 4, struct achk_opt_32)
#define ACH_CH_GET_STATUS_COMPAT        _IOR(ACH_IOCTL, 5, struct ach_ch_status_32)
#define ACH_CH_GET_OPTIONS_COMPAT       _IOW(ACH_IOCTL, 8, struct ach_ch_options_32)

#endif /* CONFIG_COMPAT */

/** ioctl argument to create channel */
struct ach_ctrl_create_ch {
	size_t frame_cnt;                   /**< Number of entries in index array */
	size_t frame_size;                  /**< Nominal size of each message */
	clockid_t clock;                    /**< Clock to use for the channel */
	char name[ACH_CHAN_NAME_MAX + 1];   /**< Name of the channel */
};

/** ioctl argument unlink channel */
struct ach_ctrl_unlink_ch {
	char name[ACH_CHAN_NAME_MAX + 1];   /**< Name of the channel */
};

/** ioctl argument get channel status. */
struct ach_ch_status {
	int mode;               /**< Get options for the channel */
	ssize_t size;		/**< Size of queue */
	ssize_t count;		/**< Messages in queue */
	ssize_t new_msgs;	/**< Unread messages in queue */
	unsigned long last_seq;	/**< Last sequence in queue */
	unsigned long last_seq_read;	/**< Sequence of last read message */
};

/** ioctl argument get channel mode and clock */
struct ach_ch_options {
	struct achk_opt mode; /* channel mode */
	clockid_t clock;         /* channel clock */
};

#ifdef CONFIG_COMPAT

/** Compat struct for ach_ctrl_create_ch */
struct ach_ctrl_create_ch_32 {
	u32 frame_cnt;                    /**< Number of entries in index array */
	u32 frame_size;                   /**< Nominal size of each message */
	clockid_t clock;                    /**< Clock to use for the channel */
	char name[ACH_CHAN_NAME_MAX + 1];   /**< Name of the channel */
};

/** Compat struct for ach_ch_status_32 */
struct ach_ch_status_32 {
	int mode;               /**< Get options for the channel */
	s32 size;             /**< Size of queue */
	s32 count;            /**< Messages in queue */
	s32 new_msgs;         /**< Unread messages in queue */
	u32 last_seq;         /**< Last sequence in queue */
	u32 last_seq_read;    /**< Sequence of last read message */
};

/** Compat struct for ach_ch_options */
struct ach_ch_options_32 {
	struct achk_opt_32 mode; /* channel mode */
	clockid_t clock;         /* channel clock */
};

#endif /* CONFIG_COMPAT */

#endif

/* Local Variables:    */
/* mode: C             */
/* c-basic-offset: 8   */
/* indent-tabs-mode: t */
/* End:                */
