#ifndef _LINUX_ACH_IPC_H_
#define _LINUX_ACH_IPC_H

/*
 * Header file for ach_ipc kernel driver
 *
 * Copyright (C) 2013, Prevas A/S
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 *
 * Authors: Kim Boendergaard Poulsen <kibo@prevas.dk>
 *
 */

#define ACH_NAME                    "ach-ipc"
#define ACH_IOCTL                   'Q'
#define ACH_MAX_DEVICES             512

#define ACH_CTRL_CREATE_CH              _IOW(ACH_IOCTL, 1, struct ach_ctrl_create_ch)
#define ACH_CTRL_UNLINK_CH              _IOW(ACH_IOCTL, 2, struct ach_ctrl_unlink_ch)


#define ACH_CH_MODE_WAIT                0x01
#define ACH_CH_MODE_LAST                0x02
#define ACH_CH_MODE_COPY                0x04

#define ACH_CH_CANCEL_UNSAFE            0x01

#define ACH_CH_SET_MODE                 _IOW(ACH_IOCTL, 3, struct ach_ch_mode)
#define ACH_CH_GET_MODE                 _IOR(ACH_IOCTL, 4, struct ach_ch_mode)
#define ACH_CH_GET_STATUS               _IOR(ACH_IOCTL, 5, struct ach_ch_status)
#define ACH_CH_FLUSH                    _IOW(ACH_IOCTL, 6, unsigned int)
#define ACH_CH_CANCEL                   _IOW(ACH_IOCTL, 7, unsigned int)

/**  maximum size of a channel name */
#define ACH_CHAN_NAME_MAX 64ul

struct ach_ctrl_create_ch {
  size_t frame_cnt;
  size_t frame_size;
  char name[ACH_CHAN_NAME_MAX+1];
};

struct ach_ctrl_unlink_ch {
  char name[ACH_CHAN_NAME_MAX+1];
};

struct ach_ch_mode {
  unsigned int mode;
  struct timespec reltime;  /* Relative time - notice ach normally runs abstime */
};

struct ach_ch_status {
  unsigned int mode;         
  ssize_t  size;             /* Size of queue */
  ssize_t  count;            /* Messages in queue */
  ssize_t  new;              /* Unread messages in queue */
  unsigned long last_seq;    /* Last sequence in queue */
  unsigned long last_seq_read; /* Sequence of last read message */
};

#endif
