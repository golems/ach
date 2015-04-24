/*
 * Kernel driver module for ach.
 *
 * Copyright (C) 2013, Prevas A/S
 * Copyright (C) 2015, Rice University
 * All rights reserved.
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

#include <linux/init.h>		/* __init and __exit macroses */
#include <linux/kernel.h>	/* KERN_INFO macros */
#include <linux/module.h>	/* required for all kernel modules */
#include <linux/moduleparam.h>	/* module_param() and MODULE_PARM_DESC() */

#include <linux/fs.h>		/* struct file_operations, struct file */
#include <linux/miscdevice.h>	/* struct miscdevice and misc_[de]register() */
#include <linux/rtmutex.h>	/* mutexes */
#include <linux/string.h>	/* memchr() function */
#include <linux/slab.h>		/* kzalloc() function */
#include <linux/sched.h>	/* wait queues */
#include <linux/uaccess.h>	/* copy_{to,from}_user() */
#include <linux/wait.h>		/* Wait queue for read processes */
#include <linux/device.h>	/* device_create() */
#include <linux/cdev.h>
#include <linux/poll.h>

#include "ach_klinux.h"
#include "ach/private_klinux.h"


#define ACH_ERRF( ... ) printk(KERN_ERR __VA_ARGS__ )

#include "ach/impl_generic.h"

/** Default number for max ach devices */
#define ACH_MAX_DEVICES             512


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Kim Bøndergaard <kibo@prevas.dk>");
MODULE_DESCRIPTION("Ach Kernel");

static unsigned max_devices = ACH_MAX_DEVICES;
module_param(max_devices, uint, (S_IRUSR | S_IRGRP | S_IROTH));
MODULE_PARM_DESC(max_devices, "Max number of ach kernel devices");

#define KDEBUG_ENABLED

#ifdef KDEBUG_ENABLED

#define KDEBUG(...) {printk (KERN_INFO __VA_ARGS__); }
#else
#define KDEBUG(...)
#endif



/* The struct controlling /dev/achctrl */
struct ach_ctrl_device {
	struct rt_mutex lock;
	struct ach_ch_device *devices;   /* array of all channel devices */
	struct ach_ch_device *free;      /* linked list of unused channel devices */
	struct ach_ch_device *in_use;    /* linked list of currently used channel devices */
	int in_use_cnt;
	int major;		/* Major number assigned to ach channel devices */
	struct class *ach_ch_class;
};

/* The static entry to all objects used by the module */
static struct ach_ctrl_device ctrl_data;

/**********************************************************************************
 * ach module stuff - ought be comparable to user space ach.c functions
 **********************************************************************************/

/* The current locking implementation only allows a single reader or
 * writer to access the channel at a time.  For the expected use case
 * of short messages, this is tolerable.  Still, performance could be
 * improved by allowing multiple readers, or even better by allowing
 * multiple readers in parallel with multiple writers operating on
 * different index array elements and data array ranges.
 */

static enum ach_status
chan_lock( ach_channel_t *chan )
{
	int i = rt_mutex_lock_interruptible(&chan->shm->sync.mutex, 0);
	if( -EINTR == i ) return ACH_EINTR;
	if( i ) return ACH_BUG;
	if( chan->cancel ) {
		rt_mutex_unlock(&chan->shm->sync.mutex);
		return ACH_CANCELED;
	}
	if( chan->shm->sync.dirty ) {
		rt_mutex_unlock(&chan->shm->sync.mutex);
		ACH_ERRF("ach bug: channel dirty on lock acquisition\n");
		return ACH_CORRUPT;
	}
	return ACH_OK;
}


static enum ach_status
rdlock_wait(ach_channel_t * chan, const struct timespec *reltime)
{
	int res;
	struct ach_header *shm = chan->shm;
	volatile uint64_t *c_seq = &chan->seq_num, *s_seq = &shm->last_seq;
	volatile unsigned int *cancel = &chan->cancel;
	enum ach_status r;

	for(;;) {
		/* do the wait */
		if (reltime->tv_sec != 0 || reltime->tv_nsec != 0) {
			res = wait_event_interruptible_timeout( shm->sync. readq,
								((*c_seq != *s_seq) || *cancel),
								timespec_to_jiffies (reltime) );
			if (0 == res) return ACH_TIMEOUT;
		} else {
			res = wait_event_interruptible( shm->sync.readq,
							((*c_seq != *s_seq) || *cancel) );
		}

		/* check what happened */
		if (-ERESTARTSYS == res) return ACH_EINTR;
		if( res < 0 ) {
			ACH_ERRF("ach bug: rdlock_wait(), "
				 "could not wait for event, "
				 "timeout: (%lu,%ld), result=%d\n",
				 reltime->tv_sec, reltime->tv_nsec, res);
			return ACH_BUG;
		}

		r = chan_lock( chan );
		/* Check condition with the lock held in case someone
		 * else flushed the channel, or someone else unset the
		 * cancel */
		if( (*c_seq != *s_seq) || (ACH_OK != r) || *cancel ) {
			return r;
		}
		rt_mutex_unlock(&shm->sync.mutex);
	}
}

static enum ach_status
rdlock(ach_channel_t * chan, int wait, const struct timespec *reltime)
{
	if( wait ) return rdlock_wait(chan, reltime);
	else return chan_lock(chan);
}

static enum ach_status unrdlock(struct ach_header *shm)
{
	int dirty = shm->sync.dirty;
	rt_mutex_unlock(&shm->sync.mutex);
	if( dirty ) {
		ACH_ERRF("ach bug: channel dirty on read unlock\n");
		return ACH_CORRUPT;
	}
	return ACH_OK;
}

static enum ach_status wrlock(ach_channel_t * chan)
{
	enum ach_status r = chan_lock(chan) ;
	if( ACH_OK != r ) return r;
	chan->shm->sync.dirty = 1;
	return ACH_OK;
}

static enum ach_status unwrlock(struct ach_header *shm)
{
	int dirty = shm->sync.dirty;
	shm->sync.dirty = 0;
	rt_mutex_unlock(&shm->sync.mutex);
	wake_up_all(&shm->sync.readq);
	if( !dirty ) {
		ACH_ERRF("ach bug: channel not dirty on write unlock\n");
		return ACH_CORRUPT;
	}
	return ACH_OK;
}

static void
ach_shm_release ( struct kref *ref )
{
	struct ach_header *shm = container_of( ref, struct ach_header, refcount );
	printk( KERN_INFO "ach: released channel %s\n", shm->name );
	kfree( shm );
}

static struct ach_header *ach_create(const char *name, size_t frame_cnt, size_t frame_size, clockid_t clock)
{
	struct ach_header *shm;
	int len = ach_create_len(frame_cnt, frame_size);

	len = sizeof(struct ach_header) +
	    frame_cnt * sizeof(ach_index_t) +
	    frame_cnt * frame_size + 3 * sizeof(uint64_t);

	shm = (struct ach_header *)kzalloc(len, GFP_KERNEL);
	if (unlikely(!shm)) {
		printk(KERN_ERR "ach: Unable to allocate buffer memory\n");
		return NULL;
	}

	shm->len = len;

	/* initialize mutex */
	rt_mutex_init(&shm->sync.mutex);
	init_waitqueue_head(&shm->sync.readq);

	/* set up refcounting  */
	kref_init(&shm->refcount);
	rt_mutex_init(&shm->ref_mutex);

	shm->clock = clock;

	/* initialize counts */
	ach_create_counts( shm, name, frame_cnt, frame_size );

	return shm;
}

static enum ach_status put_fun(void *cx, void *chan_dst, const void *obj)
{
	size_t len = *(size_t *) cx;
	if (copy_from_user(chan_dst, obj, len)) {
		return ACH_EFAULT;
	}
	return ACH_OK;
}

static enum ach_status
ach_put(ach_channel_t * chan, const void *buf, size_t len)
{
	return ach_xput(chan, put_fun, &len, buf, len);
}

static enum ach_status
get_fun(void *cx, void **obj_dst, const void *chan_src, size_t frame_size)
{
	size_t size = *(size_t *) cx;

	if (size < frame_size)
		return ACH_OVERFLOW;
	if (NULL == *obj_dst && 0 != frame_size)
		return ACH_EINVAL;
	if (copy_to_user(*obj_dst, chan_src, frame_size))
		return ACH_EFAULT;

	return ACH_OK;
}

static enum ach_status
ach_get(ach_channel_t * chan, void *buf, size_t size, size_t * frame_size)
{
	return ach_xget(chan, get_fun, &size, &buf, frame_size,
			&chan->mode.reltime, chan->mode.options);
}

static enum ach_status ach_flush(ach_channel_t * chan)
{

	return ach_flush_impl(chan);
}

static enum ach_status ach_cancel(ach_channel_t * chan, unsigned int unsafe)
{
	(void)unsafe;

	chan->cancel = 1;
	wake_up_all(&chan->shm->sync.readq);

	return ACH_OK;
}

/**********************************************************************************
 * ach channel file object
 **********************************************************************************/

/* Expects device to be locked */
struct ach_ch_file *ach_ch_file_alloc(struct ach_ch_device *device)
{
	struct ach_ch_file *ch_file;

	ch_file = kzalloc(sizeof(struct ach_ch_file), GFP_KERNEL);
	if (unlikely(!ch_file)) {
		printk(KERN_ERR "ach: Failed alloc'ing\n");
		return NULL;
	}

	kref_get(&device->ach_data->refcount);
	ch_file->shm = device->ach_data;
	ch_file->mode.options = ACH_O_WAIT;	/* Default mode */
	ch_file->mode.reltime.tv_sec = 0;
	ch_file->mode.reltime.tv_nsec = 0;
	ch_file->seq_num = 0;
	ch_file->next_index = ch_file->shm->index_head;

	return ch_file;
}

/**********************************************************************************
 * ach channel device object
 **********************************************************************************/

static void
ach_ch_make_device_name( const char *channel_name, char *device_name )
{
	strcpy( device_name, ACH_CHAR_CHAN_NAME_PREFIX_NAME);
	strcat( device_name, channel_name );
}

static struct ach_ch_device *ach_ch_devices_alloc(void)
{
	int i;
	struct ach_ch_device *devs = NULL;

	devs = kzalloc(sizeof(struct ach_ch_device) * max_devices, GFP_KERNEL);
	if (unlikely(!devs)) {
		printk(KERN_ERR "ach: Failed alloc'ing device memory\n");
		goto out;
	}

	for (i = 0; i < max_devices; i++) {
		struct ach_ch_device *device = &devs[i];
		device->minor = i;
		device->next = device + 1;	/* Form list of free devices */
		device->ach_data = NULL;
	}
	devs[max_devices - 1].next = NULL;	/* Terminate free list */

	return devs;

 out:
	return NULL;
}

static struct ach_ch_device *ach_ch_device_find(const char *channel_name)
{
	struct ach_ch_device *dev = ctrl_data.in_use;

	char device_name[ACH_CHAN_NAME_MAX + 16];
	ach_ch_make_device_name( channel_name, device_name );

	while (dev) {
		if (0 == strncmp(device_name, ach_ch_device_name(dev), sizeof(device_name)))
			return dev;
		dev = dev->next;
	}
	return NULL;
}

/* ctrl_data is expected to be locked */
static int ach_ch_device_free(struct ach_ch_device *dev)
{
	struct ach_ch_device *prev = ctrl_data.in_use;

	/* destroy devices */
	if( dev->device && dev->minor >= 0 ) {
		device_destroy(ctrl_data.ach_ch_class,
			       MKDEV(ctrl_data.major, dev->minor));
		cdev_del(&dev->cdev);
	}
	dev->device = NULL;

	/* Remove from in_use list */
	if (prev == dev) {
		ctrl_data.in_use = dev->next;
	} else {
		while (prev && prev->next != dev) {
			prev = prev->next;
		}
		if (unlikely(NULL == prev)) {
			return -ERESTARTSYS;
		}
		prev->next = dev->next;
	}
	ctrl_data.in_use_cnt--;

	/* dec refcount */
	if( dev->ach_data ) {
		kref_put( &dev->ach_data->refcount, ach_shm_release );
		dev->ach_data = NULL;
	}

	/* Insert in to free list */
	dev->next = ctrl_data.free;
	ctrl_data.free = dev;

	return 0;
}

/* ctrl_data is expected to be locked */
static int ach_ch_device_alloc(const char *channel_name)
{
	struct ach_ch_device *dev;
	char device_name[ACH_CHAN_NAME_MAX + 16];

	if (!ctrl_data.free) {
		return -ENOSR;
	}

	ach_ch_make_device_name( channel_name, device_name );

	/* Check for name clashes */
	dev = ach_ch_device_find( channel_name );
	if( dev ) return -EEXIST;

	/* So far -> Name is unique */
	/* Take first free from free list ... */
	dev = ctrl_data.free;
	ctrl_data.free = dev->next;

	/* .. and put in start of in_use list */
	dev->next = ctrl_data.in_use;
	ctrl_data.in_use = dev;
	ctrl_data.in_use_cnt++;

	//KDEBUG2("Alocated device %d for channel %s\n", dev->minor, dev->name);
	return 0;
}

static int ach_ch_devices_free_all(void)
{
	int result = 0;
	struct ach_ch_device *device = ctrl_data.in_use;

	while (device) {
		struct ach_ch_device *next = device->next;
		ach_ch_device_free(device);
		device = next;
	}

	kfree(ctrl_data.devices);

	return result;
}

/**********************************************************************************
 * ach channel device driver
 **********************************************************************************/
static int ach_ch_close(struct inode *inode, struct file *file)
{
	struct ach_ch_file *ch_file;
	int ret = 0;

	KDEBUG("ach: in ach_ch_close (inode %d)\n", iminor(inode));

	/* Synchronize to protect refcounting */
	if (rt_mutex_lock_interruptible(&ctrl_data.lock, 1)) {
		ret = -ERESTARTSYS;
		goto out;
	}

	ch_file = (struct ach_ch_file *)file->private_data;
	kref_put( &ch_file->shm->refcount, ach_shm_release );
	kfree(ch_file);

	rt_mutex_unlock(&ctrl_data.lock);

  out:
	return ret;
}

static int ach_ch_open(struct inode *inode, struct file *file)
{
	int ret = 0;
	struct ach_ch_device *device;

	/* Synchronize to protect refcounting */
	if (rt_mutex_lock_interruptible(&ctrl_data.lock, 1)) {
		ret = -ERESTARTSYS;
		goto out;
	}

	device = &ctrl_data.devices[iminor(inode)];

	if (unlikely(device->minor != iminor(inode))) {
		printk(KERN_ERR "ach: Internal data problem\n");
		ret = -ERESTARTSYS;
		goto out_unlock;
	}

	file->private_data = ach_ch_file_alloc(device);

	if (!file->private_data) {
		printk(KERN_ERR "ach: Failed allocating file data\n");
		ret = -ENOBUFS;
		goto out_unlock;
	}

	KDEBUG( "ach: opened device %s\n", ach_ch_device_name(device) );

 out_unlock:
	rt_mutex_unlock(&ctrl_data.lock);
 out:
	return ret;
}

static long ach_ch_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	/* TODO: Validate argument */
	int ret = 0;
	struct ach_ch_file *ch_file = (struct ach_ch_file *)file->private_data;

	KDEBUG("ach: In ach_ch_ioctl\n");

	switch (cmd) {

	case ACH_CH_SET_MODE: {
		struct achk_opt opt;
		if (copy_from_user(&opt, (void*)arg, sizeof(opt)) ) {
			ret = -EFAULT;
		} else {
			/* This is not threadsafe */
			ch_file->mode = opt;
			/* if (ch_file->mode.reltime.tv_sec != 0 */
			/*     || ch_file->mode.reltime.tv_nsec != 0) */
			/*	KDEBUG("ach: Setting wait time to %ld.%09ld\n", */
			/*	       ch_file->mode.reltime.tv_sec, */
			/*	       ch_file->mode.reltime.tv_nsec); */
			/* KDEBUG("ach: Got cmd ACH_CH_SET_MODE: \n"); */
			/* KDEBUG1("    ACH_O_WAIT=%d\n", */
			/*	ch_file->mode.mode & ACH_O_WAIT); */
			/* KDEBUG1("    ACH_O_LAST=%d\n", */
			/*	ch_file->mode.mode & ACH_O_LAST); */
			/* KDEBUG1("    ACH_O_COPY=%d\n", */
			/*	ch_file->mode.mode & ACH_O_COPY); */
			ret = 0;
			break;
		}
	}
	case ACH_CH_GET_MODE:{
			KDEBUG("ach: Got cmd ACH_CH_GET_MODE: %ld\n", arg);
			if( copy_to_user((void*)arg, &ch_file->mode, sizeof(ch_file->mode)) )
				ret = -EFAULT;
			else
				ret = 0;
			break;
		}

	case ACH_CH_GET_STATUS:{
			KDEBUG("ach: Got cmd ACH_CH_GET_STATUS\n");
			if (rt_mutex_lock_interruptible(&ch_file->shm->sync.mutex, 0)) {
				ret = -ERESTARTSYS;
				break;
			}

			{
				struct ach_ch_status stat;
				struct ach_header *shm = ch_file->shm;
				ach_index_t *index_ar = ACH_SHM_INDEX(shm);
				size_t oldest_index = oldest_index_i(shm);
				uint64_t oldest_seq =
				    index_ar[oldest_index].seq_num;

				stat.mode = ch_file->mode.options;
				stat.size = shm->len;
				stat.count = shm->index_cnt - shm->index_free;

				if (oldest_seq > ch_file->seq_num) {
					stat.new_msgs = shm->last_seq - oldest_seq;
				} else {
					stat.new_msgs =
					    shm->last_seq - ch_file->seq_num;
				}

				stat.last_seq = shm->last_seq;
				stat.last_seq_read = ch_file->seq_num;
				printk(KERN_INFO "ach: Status:\n");
				printk(KERN_INFO "ach:            mode : %02x\n",
				       stat.mode);
				printk(KERN_INFO "ach:            size : %zu\n",
				       stat.size);
				printk(KERN_INFO "ach:            count: %zu\n",
				       stat.count);
				printk(KERN_INFO "ach:            new  : %zu\n",
				       stat.new_msgs);
				printk(KERN_INFO "ach:   last_seq      : %lu\n",
				       stat.last_seq);
				printk(KERN_INFO "ach:   last_seq_read : %lu\n",
				       stat.last_seq_read);

				if (copy_to_user((void *)arg, &stat, sizeof(stat))) {
					ret = -EFAULT;
				}
			}
			rt_mutex_unlock(&ch_file->shm->sync.mutex);
		}
	case ACH_CH_FLUSH:
		KDEBUG("ach: Got cmd ACH_CH_FLUSH\n");
		ret = -get_errno( ach_flush(ch_file) );
		break;
	case ACH_CH_CANCEL:{
			unsigned int unsafe = (unsigned int)arg;
			KDEBUG("ach: Got cmd ACH_CH_CANCEL\n");
			ret = -get_errno(ach_cancel(ch_file, unsafe));
			break;
		}
	case ACH_CH_GET_OPTIONS: {
		struct ach_ch_options retval;
		retval.mode = ch_file->mode;
		retval.clock = ch_file->shm->clock;
		if( copy_to_user( (void*)arg, &retval, sizeof(retval) ) )
			ret = -EFAULT;
		else
			ret = 0;
		break;
	}
	default:
		printk(KERN_ERR "ach: Unknown ioctl option: %d\n", cmd);
		ret = -ENOSYS;
		break;
	}

	return ret;
}

#ifdef CONFIG_COMPAT

static long ach_ch_set_mode_compat_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct achk_opt_32 *arg32 = (struct achk_opt_32 *)arg;
	struct achk_opt arg64;
	struct achk_opt *p = compat_alloc_user_space(sizeof(arg64));
	int err;
	memset(&arg64, 0, sizeof(arg64));
	err = 0;
	err |= copy_from_user(&arg64.options, &arg32->options, sizeof(arg64.options));
	err |= compat_get_timespec(&arg64.reltime, &arg32->reltime);
	err |= copy_to_user(p, &arg64, sizeof(arg64));
	if (err)
		return -EFAULT;
	return ach_ch_ioctl(file, ACH_CH_SET_MODE, (unsigned long)p);
}

static long ach_ch_get_mode_compat_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct achk_opt_32 *arg32 = (struct achk_opt_32 *)arg;
	struct achk_opt arg64;
	struct achk_opt *p = compat_alloc_user_space(sizeof(arg64));
	int err;
	err = ach_ch_ioctl(file, ACH_CH_GET_MODE, (unsigned long)p);
	if (err)
		return err;
	err = 0;
	err |= copy_from_user(&arg64, p, sizeof(arg64));
	err |= copy_to_user(&arg32->options, &arg64.options, sizeof(arg64.options));
	err |= compat_put_timespec(&arg64.reltime, &arg32->reltime);
	if (err)
		return -EFAULT;
	return 0;
}

static long ach_ch_get_status_compat_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct ach_ch_status_32 *arg32 = (struct ach_ch_status_32 *)arg;
	struct ach_ch_status arg64;
	struct ach_ch_status *p = compat_alloc_user_space(sizeof(arg64));
	s32 i;
	u32 u;
	int err;
	err = ach_ch_ioctl(file, ACH_CH_GET_STATUS, (unsigned long)p);
	if (err)
		return err;
	err |= copy_from_user(&arg64, p, sizeof(arg64));
	i = (s32)arg64.size;
	err |= copy_to_user(&i, &arg32->size, sizeof(s32));
	i = (s32)arg64.count;
	err |= copy_to_user(&i, &arg32->count, sizeof(s32));
	i = (s32)arg64.new_msgs;
	err |= copy_to_user(&i, &arg32->new_msgs, sizeof(s32));
	u = (u32)arg64.last_seq;
	err |= copy_to_user(&u, &arg32->last_seq, sizeof(u32));
	u = (u32)arg64.last_seq_read;
	err |= copy_to_user(&u, &arg32->last_seq_read, sizeof(u32));
	if (err)
		return -EFAULT;
	return 0;
}

static long ach_ch_get_options_compat_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct ach_ch_options_32 *arg32 = (struct ach_ch_options_32 *)arg;
	struct ach_ch_options arg64;
	struct ach_ch_options *p = compat_alloc_user_space(sizeof(arg64));
	int err;
	err = ach_ch_ioctl(file, ACH_CH_GET_OPTIONS, (unsigned long)p);
	if (err)
		return err;
	err = 0;
	err |= copy_from_user(&arg64, p, sizeof(arg64));
	err |= copy_to_user(&arg32->mode.options, &arg64.mode.options, sizeof(arg64.mode.options));
	err |= copy_to_user(&arg32->clock, &arg64.clock, sizeof(arg64.clock));
	err |= compat_put_timespec(&arg64.mode.reltime, &arg32->mode.reltime);
	if (err)
		return -EFAULT;
	return 0;
}

static long ach_ch_compat_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	switch (cmd) {
	case ACH_CH_FLUSH:
	case ACH_CH_CANCEL:
		return ach_ch_ioctl(file, cmd, arg);
	case ACH_CH_SET_MODE_COMPAT:
		return ach_ch_set_mode_compat_ioctl(file, cmd, arg);
	case ACH_CH_GET_MODE_COMPAT:
		return ach_ch_get_mode_compat_ioctl(file, cmd, arg);
	case ACH_CH_GET_STATUS_COMPAT:
		return ach_ch_get_status_compat_ioctl(file, cmd, arg);
	case ACH_CH_GET_OPTIONS_COMPAT:
		return ach_ch_get_options_compat_ioctl(file, cmd, arg);
	default:
		return -ENOIOCTLCMD;
	}
}

#endif /* CONFIG_COMPAT */

static ssize_t ach_ch_write(struct file *file, const char *buffer, size_t len,
			    loff_t * offset)
{
	struct ach_ch_file *ch_file = (struct ach_ch_file *)file->private_data;
	enum ach_status r;

	/* KDEBUG1("In ach_ch_write (minor=%d)\n", ch_file->dev->minor); */

	if ( ACH_OK != (r = ach_put(ch_file, buffer, len)) )
		return -get_errno(r);

	/* KDEBUG2("wrote@%d %d bytes\n", ch_file->dev->minor, len); */
	return len;
}

static ssize_t ach_ch_read(struct file *file, char *buffer, size_t len,
			   loff_t * offset)
{
	enum ach_status stat;
	ssize_t retlen = -1;
	struct ach_ch_file *ch_file = (struct ach_ch_file *)file->private_data;

	/* KDEBUG1("In ach_ch_read (minor=%d)\n", ch_file->dev->minor); */

	stat = ach_get(ch_file, buffer, len, &retlen);
	switch (stat) {
	case ACH_MISSED_FRAME:
		/* TODO: how can we return this this? */
	case ACH_OK:
		break;
	case ACH_EINTR:
		return -ERESTARTSYS;
	default:
		return -get_errno(stat);
	}

	/* KDEBUG2("read@%d %d bytes\n", ch_file->dev->minor, retlen); */
	return retlen;
}

static unsigned int ach_ch_poll(struct file *file, poll_table * wait)
{
	unsigned int mask = POLLOUT | POLLWRNORM;
	struct ach_ch_file *ch_file = (struct ach_ch_file *)file->private_data;
	struct ach_header *shm = ch_file->shm;
	enum ach_status r;

	/* KDEBUG1("In ach_ch_poll (minor=%d)\n", ch_file->dev->minor); */

	/* Add ourselves wait queue */
	poll_wait(file, &shm->sync.readq, wait);

	/* Lock channel and check what happened */
	r = chan_lock(ch_file);
	if( ACH_OK != r ) return -get_errno(r);

	if (ch_file->seq_num != shm->last_seq) {
		mask |= POLLIN | POLLRDNORM;
	}

	rt_mutex_unlock(&shm->sync.mutex);

	return mask;
}

static const struct file_operations ach_ch_fops = {
	.owner = THIS_MODULE,
	.open = ach_ch_open,
	.release = ach_ch_close,
	.write = ach_ch_write,
	.read = ach_ch_read,
	.unlocked_ioctl = ach_ch_ioctl,
#ifdef CONFIG_COMPAT
    .compat_ioctl = ach_ch_compat_ioctl,
#endif /* CONFIG_COMPAT */
	.poll = ach_ch_poll,
};

static int ach_ch_device_setup(struct ach_ch_device *dev, const char *channel_name)
{
	int ret = 0;

	cdev_init(&dev->cdev, &ach_ch_fops);
	dev->cdev.owner = THIS_MODULE;

	ret = cdev_add(&dev->cdev, MKDEV(ctrl_data.major, dev->minor), 1);
	if (ret < 0) {
		printk(KERN_ERR
		       "ach: Adding device to kernel failed (minor=%u)\n",
		       dev->minor);
		return ret;
	}

	dev->device = device_create(ctrl_data.ach_ch_class, NULL,
				    MKDEV(ctrl_data.major, dev->minor),
				    NULL, ACH_CHAR_CHAN_NAME_PREFIX_NAME "%s",
				    channel_name);
	if (IS_ERR(dev->device)) {
		printk(KERN_ERR "ach: device_create failed\n");
		return PTR_ERR(dev->device);
	}


	return ret;
}

/**********************************************************************************
 * ach control device driver
 **********************************************************************************/
static int
ctrl_create (struct ach_ctrl_create_ch *arg )
{
	int ret;
	struct ach_ch_device *dev;

	ret = ach_ch_device_alloc(arg->name);
	if( unlikely(ret < 0) )
		return ret;
	// newest is first in queue
	dev = ctrl_data.in_use;

	dev->ach_data = ach_create(arg->name, arg->frame_cnt, arg->frame_size, arg->clock);

	if (unlikely(NULL == dev->ach_data)) {
		printk(KERN_ERR "ach: Unable to allocate buffer memory\n");
		return -ENOBUFS;
	}

	ret = ach_ch_device_setup(dev, arg->name);
	if( unlikely(ret < 0) ) {
		printk(KERN_ERR "ach: Failed creating char device\n");
		kfree(dev->ach_data);
		return ret;
	}

	printk( KERN_INFO "ach: created device %s\n", ach_ch_device_name(dev) );

	return ret;
}



static long ach_ctrl_ioctl(struct file *file, unsigned int cmd,
			   unsigned long arg)
{
	/* TODO: Validate argument */
	int ret = 0;

	if (rt_mutex_lock_interruptible(&ctrl_data.lock, 1)) {
		return -ERESTARTSYS;
	}

	switch (cmd) {

	case ACH_CTRL_CREATE_CH: {
		struct ach_ctrl_create_ch create_arg;
		KDEBUG("ach: Control command create\n");
		if (copy_from_user(&create_arg, (void*)arg, sizeof(create_arg)) ) {
			ret = -EFAULT;
		} else if ( strnlen(create_arg.name,ACH_CHAN_NAME_MAX)
			    >= ACH_CHAN_NAME_MAX ) {
			ret = -ENAMETOOLONG;
		} else {
			ret = ctrl_create( &create_arg );
		}
		break;
	}
	case ACH_CTRL_UNLINK_CH:{
		struct ach_ctrl_unlink_ch unlink_arg;
		KDEBUG("ach: Control command unlink\n");
		if (copy_from_user(&unlink_arg, (void*)arg, sizeof(unlink_arg)) ) {
			ret = -EFAULT;
		} else if ( strnlen(unlink_arg.name,ACH_CHAN_NAME_MAX)
			    >= ACH_CHAN_NAME_MAX ) {
			ret = -ENAMETOOLONG;
		} else {
			/* Find the device */
			struct ach_ch_device *dev;
			dev = ach_ch_device_find(unlink_arg.name);
			if (!dev) {
				ret = -ENOENT;
				goto out_unlock;
			}
			/* Free the device.  The channel
			 * backing memory is ref-counted, and
			 * won't be freed until all files are
			 * closed.*/
			if (ach_ch_device_free(dev)) {
				ret = -ERESTARTSYS;
				goto out_unlock;
			}
			printk( KERN_INFO "ach: unlinked channel %s\n", unlink_arg.name );
		}
		break;
	}
	default:
		printk(KERN_ERR "ach: Unknown ioctl option: %d\n", cmd);
		ret = -ENOSYS;
		break;
	}

 out_unlock:
	rt_mutex_unlock(&ctrl_data.lock);
	return ret;
}

#ifdef CONFIG_COMPAT

static long ach_ctrl_create_ch_compat_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct ach_ctrl_create_ch_32 *arg32 = (struct ach_ctrl_create_ch_32 *)arg;
	struct ach_ctrl_create_ch arg64;
	struct ach_ctrl_create_ch *p = compat_alloc_user_space(sizeof(arg64));
	int err;
	u32 u;
	memset(&arg64, 0, sizeof(arg64));
	err = 0;
	err |= copy_from_user(&u, &arg32->frame_cnt, sizeof(u32));
	arg64.frame_cnt = (size_t)u;
	err |= copy_from_user(&u, &arg32->frame_size, sizeof(u32));
	arg64.frame_size = (size_t)u;
	err |= copy_from_user(&arg64.clock, &arg32->clock, sizeof(arg64.clock));
	err |= copy_from_user(arg64.name, arg32->name, ACH_CHAN_NAME_MAX + 1);
	err |= copy_to_user(p, &arg64, sizeof(arg64));
	if (err)
		return -EFAULT;
	return ach_ctrl_ioctl(file, ACH_CTRL_CREATE_CH, (unsigned long)p);
}

static long ach_ctrl_compat_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	switch (cmd) {
	case ACH_CTRL_UNLINK_CH:
		return ach_ctrl_ioctl(file, cmd, arg);		
	case ACH_CTRL_CREATE_CH_COMPAT:
		return ach_ctrl_create_ch_compat_ioctl(file, cmd, arg);
	default:
		return -ENOIOCTLCMD;
	}
}

#endif /* CONFIG_COMPAT */

static struct file_operations ach_ctrl_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = ach_ctrl_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = ach_ctrl_compat_ioctl,
#endif /* CONFIG_COMPAT */
	.open = NULL,
	.release = NULL,
	.read = NULL,
	.write = NULL,
	.poll = NULL,
	.llseek = NULL,
};

static struct miscdevice ach_misc_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "achctrl",
	.fops = &ach_ctrl_fops
};

/**********************************************************************************
 * MODULE INIT
 **********************************************************************************/
static int __init ach_init(void)
{
	int ret = 0;

	if (!max_devices)
		return -1;

	/* Init ctrl data */
	ctrl_data.in_use_cnt = 0;
	ctrl_data.devices = ach_ch_devices_alloc();
	ctrl_data.free = ctrl_data.devices;
	ctrl_data.in_use = NULL;
	rt_mutex_init(&ctrl_data.lock);

	/* Create ctrl device */
	misc_register(&ach_misc_device);
	printk(KERN_INFO
	       "ach: device registered with a max of %u devices\n",
	       max_devices);

	/* Common channel device stuff */

	/* We'll use own major number as we want to control the minor numbers ourself */
	{
		dev_t dev_num;

		ctrl_data.ach_ch_class = class_create(THIS_MODULE, ACH_CH_SUBSYSTEM);
		if (IS_ERR(ctrl_data.ach_ch_class)) {
			ret = PTR_ERR(ctrl_data.ach_ch_class);
			printk(KERN_ERR "ach: Failed to create class\n");
			goto out_deregister;
		}

		/* Allocate major */
		dev_num = 0;
		ret = alloc_chrdev_region(&dev_num, 0, max_devices, ACH_CH_SUBSYSTEM);
		if (ret < 0) {
			printk(KERN_ERR
			       "ach: Failed to allocate major number with %u minor numbers\n",
			       max_devices);
			goto out_classdestroy;
		}

		ctrl_data.major = MAJOR(dev_num);
	}

	return 0;

 out_classdestroy:
	class_destroy(ctrl_data.ach_ch_class);

 out_deregister:
	misc_deregister(&ach_misc_device);
	ach_ch_devices_free_all();

	return ret;
}

static void __exit ach_exit(void)
{
	if (ach_ch_devices_free_all())
		printk(KERN_ERR "ach: failed cleaning up\n");

	class_destroy(ctrl_data.ach_ch_class);
	ctrl_data.ach_ch_class = NULL;

	unregister_chrdev_region(MKDEV(ctrl_data.major, 0), max_devices);

	misc_deregister(&ach_misc_device);
	printk(KERN_INFO "ach: device unregistered\n");
}

module_init(ach_init);
module_exit(ach_exit);

/* Local Variables:    */
/* mode: C             */
/* c-basic-offset: 8   */
/* indent-tabs-mode: t */
/* End:                */
