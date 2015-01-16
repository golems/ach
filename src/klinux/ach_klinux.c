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
 */

#include <linux/init.h>		/* __init and __exit macroses */
#include <linux/kernel.h>	/* KERN_INFO macros */
#include <linux/module.h>	/* required for all kernel modules */
#include <linux/moduleparam.h>	/* module_param() and MODULE_PARM_DESC() */

#include <linux/fs.h>		/* struct file_operations, struct file */
#include <linux/miscdevice.h>	/* struct miscdevice and misc_[de]register() */
#include <linux/mutex.h>	/* mutexes */
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


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Kim Bøndergaard <kibo@prevas.dk>");
MODULE_DESCRIPTION("Ach Kernel");

static unsigned max_devices = ACH_MAX_DEVICES;
module_param(max_devices, uint, (S_IRUSR | S_IRGRP | S_IROTH));
MODULE_PARM_DESC(max_devices, "Max number of ach kernel devices");

//#define KDEBUG_ENABLED

#ifdef KDEBUG_ENABLED
#define KDEBUG(xx) {printk (KERN_INFO xx); }
#define KDEBUG1(xx, arg1) {printk(KERN_INFO xx, (arg1));}
#define KDEBUG2(xx, arg1,arg2) {printk(KERN_INFO xx, (arg1), (arg2));}
#else
#define KDEBUG(xx)
#define KDEBUG1(xx, arg1)
#define KDEBUG2(xx, arg1,arg2)
#endif

/* The struct controlling /dev/achctrl */
struct ach_ctrl_device {
	wait_queue_head_t some_queue_if_needed;
	struct mutex lock;
	struct ach_ch_device *devices;
	struct ach_ch_device *free;
	struct ach_ch_device *in_use;
	int in_use_cnt;
	int major;		/* Major number assigned to ach channel devices */
	struct class *ach_ch_class;
};

static int ach_ch_setup_cdev(struct ach_ch_device *dev);
static int ach_ch_remove_cdev(struct ach_ch_device *dev);

/* The static entry to all objects used by the module */
static struct ach_ctrl_device ctrl_data;

/**********************************************************************************
 * ach module stuff - ought be comparable to user space ach.c functions
 **********************************************************************************/


#include "ach/impl_generic.h"

static enum ach_status
rdlock(ach_channel_t * chan, int wait, const struct timespec *reltime)
{
	// TODO: Missing a lot compared to userspace ach.c
	struct ach_header *shm = chan->shm;

	if (mutex_lock_interruptible(&shm->sync.mutex)) {
		return ACH_BUG;
	}

	{
		enum ach_status r = ACH_BUG;

		while (ACH_BUG == r) {
			if (!wait) {
				r = ACH_OK;
			} else if (chan->seq_num != shm->last_seq) {
				r = ACH_OK;
			} else if (reltime->tv_sec != 0
				   || reltime->tv_nsec != 0) {
				int res;
				mutex_unlock(&shm->sync.mutex);
				res =
				    wait_event_interruptible_timeout(shm->sync.
								     readq,
								     ((chan->
								       seq_num
								       !=
								       shm->
								       last_seq)
								      || chan->
								      cancel),
								     timespec_to_jiffies
								     (reltime));
				if (chan->cancel)
					return ACH_CANCELED;
				if (0 == res) {
					/* Timeout */
					return ACH_TIMEOUT;
				}
				if (0 > res) {
					/* SIGNAL received */
					return ACH_CANCELED;
				}
				/* > 0 => Condition is true */
				if (mutex_lock_interruptible(&shm->sync.mutex)) {
					return ACH_BUG;
				}
			} else {
				mutex_unlock(&shm->sync.mutex);
				if (wait_event_interruptible
				    (shm->sync.readq,
				     ((chan->seq_num != shm->last_seq)
				      || chan->cancel))) {
					return ACH_BUG;
				}
				if (chan->cancel) {
					return ACH_CANCELED;
				}
				if (mutex_lock_interruptible(&shm->sync.mutex)) {
					return ACH_BUG;
				}
			}
		}
	}
	return ACH_OK;
}

static enum ach_status unrdlock(struct ach_header *shm)
{
	mutex_unlock(&shm->sync.mutex);
	return ACH_OK;
}

static enum ach_status wrlock(ach_channel_t * chan)
{
	struct ach_header *shm = chan->shm;

	if (mutex_lock_interruptible(&shm->sync.mutex)) {
		return ACH_FAILED_SYSCALL;
	}
	chan->shm->sync.dirty = 1;

	return ACH_OK;
}

static enum ach_status unwrlock(struct ach_header *shm)
{
	shm->sync.dirty = 0;
	mutex_unlock(&shm->sync.mutex);

	wake_up(&shm->sync.readq);

	return ACH_OK;
}

typedef enum ach_status
ach_put_fun(void *cx, void *chan_dst, const void *obj_src);

typedef enum ach_status
ach_get_fun(void *cx, void **obj_dst, const void *chan_src, size_t frame_size);

static struct ach_header *ach_create(size_t frame_cnt, size_t frame_size)
{
	struct ach_header *shm;
	int len = ach_create_len(frame_cnt, frame_size);

	len = sizeof(struct ach_header) +
	    frame_cnt * sizeof(ach_index_t) +
	    frame_cnt * frame_size + 3 * sizeof(uint64_t);

	shm = (struct ach_header *)kzalloc(len, GFP_KERNEL);
	if (unlikely(!shm)) {
		printk(KERN_ERR "Unable to allocate buffer memory\n");
		return NULL;
	}

	shm->len = len;

	/* initialize mutex */
	mutex_init(&shm->sync.mutex);
	init_waitqueue_head(&shm->sync.readq);

	/* initialize counts */
	ach_create_counts( shm, frame_cnt, frame_size );

	return shm;
}

static enum ach_status put_fun(void *cx, void *chan_dst, const void *obj)
{
	size_t len = *(size_t *) cx;
	if (-EFAULT == copy_from_user(chan_dst, obj, len)) {
		return -ACH_FAILED_SYSCALL;
	}
	return ACH_OK;
}

#define ACH_XPUT_ASSERT(cond)						\
	{								\
		if (! (cond) ) {					\
			printk(KERN_ERR "Logic error in ACH\n");	\
			return ACH_BUG;					\
		}							\
	}

static enum ach_status
ach_xput(ach_channel_t * chan,
	 ach_put_fun transfer, void *cx, const void *obj, size_t len)
{
	struct ach_header *shm;
	ach_index_t *index_ar;
	unsigned char *data_ar;
	ach_index_t *idx;

	shm = chan->shm;

	/* Check guard bytes */
	{
		enum ach_status r = check_guards(shm);
		if (ACH_OK != r)
			return r;
	}

	if (shm->data_size < len) {
		return ACH_OVERFLOW;
	}

	index_ar = ACH_SHM_INDEX(shm);
	data_ar = ACH_SHM_DATA(shm);

	/* take write lock */
	wrlock(chan);

	/* find next index entry */
	idx = index_ar + shm->index_head;

	/* clear entry used by index */
	if (0 == shm->index_free) {
		free_index(shm, shm->index_head);
	} else {
		ACH_XPUT_ASSERT(0 == index_ar[shm->index_head].seq_num);
	}

	ACH_XPUT_ASSERT(shm->index_free > 0);

	/* Avoid wraparound */
	if (shm->data_size - shm->data_head < len) {
		size_t i;
		/* Clear to end of array */
		ACH_XPUT_ASSERT(0 == index_ar[shm->index_head].offset);
		for (i = (shm->index_head + shm->index_free) % shm->index_cnt;
		     index_ar[i].offset > shm->data_head;
		     i = (i + 1) % shm->index_cnt) {
			ACH_XPUT_ASSERT(i != shm->index_head);
			free_index(shm, i);
		}
		/* Set counts to beginning of array */
		if (i == shm->index_head) {
			shm->data_free = shm->data_size;
		} else {
			shm->data_free = index_ar[oldest_index_i(shm)].offset;
		}
		shm->data_head = 0;
	}

	/* clear overlapping entries */
	{
		size_t i;
		for (i = (shm->index_head + shm->index_free) % shm->index_cnt;
		     shm->data_free < len; i = (i + 1) % shm->index_cnt) {
			if (i == shm->index_head) {
				shm->data_free = shm->data_size;
			} else {
				free_index(shm, i);
			}
		}

		ACH_XPUT_ASSERT(shm->data_free >= len);

		if (shm->data_size - shm->data_head < len) {
			unwrlock(shm);
			return ACH_BUG;
		}
	}

	/* transfer */
	{
		enum ach_status r = transfer(cx, data_ar + shm->data_head, obj);
		if (ACH_OK != r) {
			unwrlock(shm);
			return r;
		}
	}

	/* modify counts */
	shm->last_seq++;
	idx->seq_num = shm->last_seq;
	idx->size = len;
	idx->offset = shm->data_head;

	shm->data_head = (shm->data_head + len) % shm->data_size;
	shm->data_free -= len;
	shm->index_head = (shm->index_head + 1) % shm->index_cnt;
	shm->index_free--;

	ACH_XPUT_ASSERT(shm->index_free <= shm->index_cnt);
	ACH_XPUT_ASSERT(shm->data_free <= shm->data_size);
	ACH_XPUT_ASSERT(shm->last_seq > 0);

	/* release write lock */
	return unwrlock(shm);
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
	if (-EFAULT == copy_to_user(*obj_dst, chan_src, frame_size))
		return ACH_FAILED_SYSCALL;

	return ACH_OK;
}

static enum ach_status
ach_xget_from_offset(ach_channel_t * chan, size_t index_offset,
		     ach_get_fun transfer, void *cx, void **pobj,
		     size_t * frame_size)
{
	struct ach_header *shm;
	ach_index_t *idx;

	shm = chan->shm;
	idx = ACH_SHM_INDEX(shm) + index_offset;

	if (chan->seq_num > idx->seq_num) {
		printk(KERN_ERR "ach bug: seq_num mismatch\n");
		return ACH_BUG;
	}

	if (idx->offset + idx->size > shm->data_size) {
		return ACH_CORRUPT;
	}

	/* good to copy */
	{
		enum ach_status r;
		unsigned char *data_buf = ACH_SHM_DATA(shm);
		*frame_size = idx->size;
		r = transfer(cx, pobj, data_buf + idx->offset, idx->size);
		if (ACH_OK == r) {
			chan->seq_num = idx->seq_num;
			chan->next_index = (index_offset + 1) % shm->index_cnt;
		}
		return r;
	}
}

static enum ach_status
ach_xget(ach_channel_t * chan, ach_get_fun transfer, void *cx, void **pobj,
	 size_t * frame_size)
{
	struct ach_header *shm;
	bool missed_frame = 0;
	enum ach_status retval = ACH_BUG;
	const bool o_wait = chan->mode.mode & ACH_O_WAIT;
	const bool o_last = chan->mode.mode & ACH_O_LAST;
	const bool o_copy = chan->mode.mode & ACH_O_COPY;
	const struct timespec *reltime = &chan->mode.reltime;

	/* take read lock */
	{
		enum ach_status r = rdlock(chan, o_wait, reltime);
		if (ACH_OK != r)
			return r;
	}

	shm = chan->shm;
	/* get the data */
	if ((chan->seq_num == shm->last_seq && !o_copy) || 0 == shm->last_seq) {
		/* no entries */
		retval = ACH_STALE_FRAMES;
	} else {
		/* Compute the index to read */
		size_t read_index;
		ach_index_t *index_ar = ACH_SHM_INDEX(shm);
		if (o_last) {
			/* normal case, get last */
			read_index = last_index_i(shm);
		} else if (!o_last &&
			   index_ar[chan->next_index].seq_num ==
			   chan->seq_num + 1) {
			/* normal case, get next */
			read_index = chan->next_index;
		} else {
			/* exception case, figure out which frame */
			if (chan->seq_num == shm->last_seq) {
				/* copy last */
				read_index = last_index_i(shm);
			} else {
				/* copy oldest */
				read_index = oldest_index_i(shm);
			}
		}

		if (index_ar[read_index].seq_num > chan->seq_num + 1) {
			missed_frame = 1;
		}

		/* read from the index */
		retval =
		    ach_xget_from_offset(chan, read_index, transfer, cx, pobj,
					 frame_size);
	}

	/* relase read lock */
	{
		ach_status_t r = unrdlock(shm);
		if (ACH_OK != r)
			return r;
	}

	return (ACH_OK == retval && missed_frame) ? ACH_MISSED_FRAME : retval;
}

static enum ach_status
ach_get(ach_channel_t * chan, void *buf, size_t size, size_t * frame_size)
{
	return ach_xget(chan, get_fun, &size, &buf, frame_size);
}

static enum ach_status ach_flush(ach_channel_t * chan)
{

	return ach_flush_impl(chan);
}

static enum ach_status ach_cancel(ach_channel_t * chan, unsigned int unsafe)
{
	if (!unsafe) {
		enum ach_status r = wrlock(chan);
		if (ACH_OK != r) {
			printk(KERN_WARNING
			       "Read lock failed => Unsafe cancel\n");
			unsafe = 1;
		}
	}

	chan->cancel = 1;

	if (!unsafe) {
		unwrlock(chan->shm);
	} else {
		wake_up(&chan->shm->sync.readq);
	}
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
		printk(KERN_ERR "Failed alloc'ing\n");
		return NULL;
	}

	ch_file->dev = device;
	ch_file->shm = device->ach_data;
	ch_file->mode.mode = ACH_O_WAIT;	/* Default mode */
	ch_file->seq_num = 0;
	ch_file->next_index = ch_file->shm->index_head;

	device->open_files++;

	return ch_file;
}

/* Expects device to be locked */
void ach_ch_file_free(struct ach_ch_file *ch_file)
{
	ch_file->dev->open_files--;
	kfree(ch_file);
}

/**********************************************************************************
 * ach channel device object
 **********************************************************************************/
static struct ach_ch_device *ach_ch_devices_alloc(void)
{
	int i;
	struct ach_ch_device *devs = NULL;

	devs = kzalloc(sizeof(struct ach_ch_device) * max_devices, GFP_KERNEL);
	if (unlikely(!devs)) {
		printk(KERN_ERR "Failed alloc'ing device memory\n");
		goto out;
	}

	for (i = 0; i < max_devices; i++) {
		struct ach_ch_device *device = &devs[i];
		mutex_init(&device->lock);
		device->minor = i;
		device->name = NULL;
		device->next = device + 1;	/* Form list of free devices */
	}
	devs[max_devices - 1].next = NULL;	/* Terminate free list */

	return devs;

 out:
	return NULL;
}

static int ach_ch_device_init(struct ach_ch_device *dev,
			      size_t frame_cnt, size_t frame_size)
{
	int ret = 0;

	dev->ach_data = ach_create(frame_cnt, frame_size);
	if (unlikely(!dev->ach_data)) {
		printk(KERN_ERR "Unable to allocate buffer memory\n");
		return -ENOBUFS;
	}

	ret = ach_ch_setup_cdev(dev);
	if (ret < 0) {
		printk(KERN_INFO "Failed creating char device\n");
		goto out_err;
	}

	return 0;

 out_err:
	kfree(dev->ach_data);

	return ret;
}

static struct ach_ch_device *ach_ch_device_find(const char *name)
{
	struct ach_ch_device *dev = ctrl_data.in_use;

	while (dev) {
		if (0 == strcmp(name, dev->name))
			return dev;
		dev = dev->next;
	}
	return NULL;
}

/* ctrl_data is expected to be locked */
static int ach_ch_device_free(struct ach_ch_device *dev)
{
	struct ach_ch_device *prev = ctrl_data.in_use;

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

	/* Insert in to free list */
	dev->next = ctrl_data.free;
	ctrl_data.free = dev;

	/* Cleanup */
	kfree(dev->name);
	dev->name = NULL;

	return 0;
}

/* ctrl_data is expected to be locked */
static int ach_ch_device_alloc(const char *name)
{
	struct ach_ch_device *dev;

	if (!ctrl_data.free) {
		return -ENOSR;
	}

	/* Check for name clashes */
	dev = ctrl_data.in_use;
	while (dev) {
		if (0 == strcmp(name, dev->name)) {
			return -EADDRINUSE;
		}
		dev = dev->next;
	}

	/* So far -> Name is unique */
	/* Take first free from free list ... */
	dev = ctrl_data.free;
	ctrl_data.free = dev->next;

	dev->name = kzalloc(strlen(name) + 1, GFP_KERNEL);
	if (unlikely(!dev->name)) {
		printk(KERN_ERR "Couldn't alloc space for device name\n");

		//TODO: Put object back on free list
		return -ENOBUFS;
	}

	strcpy(dev->name, name);

	/* .. and put in start of in_use list */
	dev->next = ctrl_data.in_use;
	ctrl_data.in_use = dev;
	ctrl_data.in_use_cnt++;

	KDEBUG2("Alocated device %d for channel %s\n", dev->minor, dev->name);
	return 0;
}

static int ach_ch_devices_free_all(void)
{
	int result = 0;
	struct ach_ch_device *device = ctrl_data.in_use;

	while (device) {
		struct ach_ch_device *next = device->next;
		ach_ch_device_free(device);
		ach_ch_remove_cdev(device);

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
	struct ach_ch_device *device;

	int ret = 0;

	KDEBUG1("In ach_ch_close (inode %d)\n", iminor(inode));

	ch_file = (struct ach_ch_file *)file->private_data;
	device = ch_file->dev;

	if (mutex_lock_interruptible(&device->lock)) {
		ret = -ERESTARTSYS;
		goto out;
	}

	ach_ch_file_free(ch_file);

	mutex_unlock(&device->lock);

 out:
	return ret;
}

static int ach_ch_open(struct inode *inode, struct file *file)
{
	struct ach_ch_device *device;
	int ret = 0;

	device = &ctrl_data.devices[iminor(inode)];
	if (unlikely(device->minor != iminor(inode))) {
		printk(KERN_ERR "Internal data problem\n");
		return -ERESTARTSYS;
	}

	if (mutex_lock_interruptible(&device->lock)) {
		ret = -ERESTARTSYS;
		goto out;
	}

	file->private_data = ach_ch_file_alloc(device);

	if (!file->private_data) {
		printk(KERN_INFO "Failed allocating file data\n");
		ret = -ENOBUFS;
		goto out_unlock;
	}

 out_unlock:
	mutex_unlock(&device->lock);

 out:
	return ret;
}

static long ach_ch_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	struct ach_ch_file *ch_file = (struct ach_ch_file *)file->private_data;
	struct ach_ch_device *dev = ch_file->dev;

	KDEBUG("In ach_ch_ioctl\n");
	switch (cmd) {

	case ACH_CH_SET_MODE:{

			ch_file->mode = *(struct ach_ch_mode *)arg;
			if (ch_file->mode.reltime.tv_sec != 0
			    || ch_file->mode.reltime.tv_nsec != 0)
				KDEBUG2("Setting wait time to %ld.%09ld\n",
					ch_file->mode.reltime.tv_sec,
					ch_file->mode.reltime.tv_nsec);
			KDEBUG("Got cmd ACH_CH_SET_MODE: \n");
			KDEBUG1("    ACH_O_WAIT=%d\n",
				ch_file->mode.mode & ACH_O_WAIT);
			KDEBUG1("    ACH_O_LAST=%d\n",
				ch_file->mode.mode & ACH_O_LAST);
			KDEBUG1("    ACH_O_COPY=%d\n",
				ch_file->mode.mode & ACH_O_COPY);
			goto out;
			break;
		}

	case ACH_CH_GET_MODE:{
			KDEBUG1("Got cmd ACH_CH_GET_MODE: %ld\n", arg);
			*(struct ach_ch_mode *)arg = ch_file->mode;
			goto out;
			break;
		}

	case ACH_CH_GET_STATUS:{
			KDEBUG("Got cmd ACH_CH_GET_STATUS\n");
			// TODO: Lock shm instead
			if (mutex_lock_interruptible(&dev->lock)) {
				ret = -ERESTARTSYS;
				goto out;
			}

			{
				struct ach_ch_status stat;
				struct ach_header *shm = ch_file->shm;
				ach_index_t *index_ar = ACH_SHM_INDEX(shm);
				size_t oldest_index = oldest_index_i(shm);
				uint64_t oldest_seq =
				    index_ar[oldest_index].seq_num;

				stat.mode = ch_file->mode.mode;
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
				printk(KERN_INFO "Status:\n");
				printk(KERN_INFO "           mode : %02x\n",
				       stat.mode);
				printk(KERN_INFO "           size : %zu\n",
				       stat.size);
				printk(KERN_INFO "           count: %zu\n",
				       stat.count);
				printk(KERN_INFO "           new  : %zu\n",
				       stat.new_msgs);
				printk(KERN_INFO "  last_seq      : %lu\n",
				       stat.last_seq);
				printk(KERN_INFO "  last_seq_read : %lu\n",
				       stat.last_seq_read);

				if (-EFAULT ==
				    copy_to_user((void *)arg, &stat,
						 sizeof(stat))) {
					ret = -EFAULT;
				}
			}
			break;
		}

	case ACH_CH_FLUSH:
		KDEBUG("Got cmd ACH_CH_FLUSH\n");
		if (ACH_OK != ach_flush(ch_file))
			ret = -ERESTARTSYS;
		break;

	case ACH_CH_CANCEL:{
			unsigned int unsafe = (unsigned int)arg;
			KDEBUG("Got cmd ACH_CH_CANCEL\n");

			if (ACH_OK != ach_cancel(ch_file, unsafe))
				ret = -ERESTARTSYS;
			break;
		}

	default:
		printk(KERN_INFO "Unknown ioctl option: %d\n", cmd);
		ret = -ENOSYS;
		break;
	}

	mutex_unlock(&dev->lock);
 out:
	return ret;
}

static ssize_t ach_ch_write(struct file *file, const char *buffer, size_t len,
			    loff_t * offset)
{
	struct ach_ch_file *ch_file = (struct ach_ch_file *)file->private_data;

	KDEBUG1("In ach_ch_write (minor=%d)\n", ch_file->dev->minor);

	if (ACH_OK != ach_put(ch_file, buffer, len)) {
		return -EFAULT;
	}
	KDEBUG2("wrote@%d %d bytes\n", ch_file->dev->minor, len);
	return len;
}

static ssize_t ach_ch_read(struct file *file, char *buffer, size_t len,
			   loff_t * offset)
{
	enum ach_status stat;
	ssize_t retlen = -1;
	struct ach_ch_file *ch_file = (struct ach_ch_file *)file->private_data;

	KDEBUG1("In ach_ch_read (minor=%d)\n", ch_file->dev->minor);

	stat = ach_get(ch_file, buffer, len, &retlen);
	switch (stat) {
	case ACH_OK:
		break;
	case ACH_TIMEOUT:
		return -ETIME;
	case ACH_MISSED_FRAME:
		break;
	case ACH_CANCELED:
		return -ECANCELED;
	case ACH_STALE_FRAMES:
		return -EAGAIN;
	default:
		return -ERESTARTSYS;
	}

	KDEBUG2("read@%d %d bytes\n", ch_file->dev->minor, retlen);
	return retlen;
}

static unsigned int ach_ch_poll(struct file *file, poll_table * wait)
{
	unsigned int mask;
	struct ach_ch_file *ch_file = (struct ach_ch_file *)file->private_data;
	struct ach_header *shm = ch_file->shm;

	KDEBUG1("In ach_ch_poll (minor=%d)\n", ch_file->dev->minor);

	poll_wait(file, &shm->sync.readq, wait);
	mask = POLLOUT | POLLWRNORM;
	if (!mutex_lock_interruptible(&shm->sync.mutex)) {
		if (ch_file->seq_num != shm->last_seq) {
			mask |= POLLIN | POLLRDNORM;
		}
		mutex_unlock(&shm->sync.mutex);
	}

	return mask;
}

static const struct file_operations ach_ch_fops = {
	.owner = THIS_MODULE,
	.open = ach_ch_open,
	.release = ach_ch_close,
	.write = ach_ch_write,
	.read = ach_ch_read,
	.unlocked_ioctl = ach_ch_ioctl,
	.poll = ach_ch_poll,
};

static int ach_ch_setup_cdev(struct ach_ch_device *dev)
{
	struct device *ret_dev;
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

	ret_dev = device_create(ctrl_data.ach_ch_class, NULL,
				MKDEV(ctrl_data.major, dev->minor),
				NULL, "%s%s",
				ACH_CHAR_CHAN_NAME_PREFIX_NAME, dev->name);
	if (IS_ERR(ret_dev)) {
		printk(KERN_ERR "device_create failed\n");
		return PTR_ERR(ret_dev);
	}

	return ret;
}

static int ach_ch_remove_cdev(struct ach_ch_device *dev)
{
	device_destroy(ctrl_data.ach_ch_class,
		       MKDEV(ctrl_data.major, dev->minor));
	cdev_del(&dev->cdev);

	return 0;
}

/**********************************************************************************
 * ach control device driver
 **********************************************************************************/
static long ach_ctrl_ioctl(struct file *file, unsigned int cmd,
			   unsigned long arg)
{
	int ret = 0;

	switch (cmd) {

	case ACH_CTRL_CREATE_CH:{
			struct ach_ctrl_create_ch *create_arg =
			    (struct ach_ctrl_create_ch *)arg;

			if (mutex_lock_interruptible(&ctrl_data.lock)) {
				ret = -ERESTARTSYS;
				goto out;
			}

			ret = ach_ch_device_alloc(create_arg->name);
			if (ret) {
				goto out_unlock;
			}

			{
				// newest is first in queue
				struct ach_ch_device *dev = ctrl_data.in_use;

				ret =
				    ach_ch_device_init(dev,
						       create_arg->frame_cnt,
						       create_arg->frame_size);
				if (ret) {
					ach_ch_device_free(dev);
				}
			}
			break;
		}

	case ACH_CTRL_UNLINK_CH:{
			struct ach_ctrl_unlink_ch *unlink_arg =
			    (struct ach_ctrl_unlink_ch *)arg;

			if (mutex_lock_interruptible(&ctrl_data.lock)) {
				ret = -ERESTARTSYS;
				goto out;
			}

			{
				struct ach_ch_device *dev;
				dev = ach_ch_device_find(unlink_arg->name);
				if (!dev) {
					ret = -ENOSTR;
					goto out_unlock;
				}

				if (mutex_lock_interruptible(&dev->lock)) {
					ret = -ERESTARTSYS;
					goto out_unlock;
				}
				//TODO: How to prevent other threads from using it afterward?
				//TODO: Can we remove it if there are open filehandles?
				if (ach_ch_device_free(dev)) {
					ret = -ERESTARTSYS;
					mutex_unlock(&dev->lock);
					goto out_unlock;
				}

				if (ach_ch_remove_cdev(dev)) {
					ret = -ERESTARTSYS;
					mutex_unlock(&dev->lock);
					goto out_unlock;
				}

				mutex_unlock(&dev->lock);
			}
			break;
		}

	default:
		printk(KERN_INFO "Unknown ioctl option: %d\n", cmd);
		ret = -ENOSYS;
		break;
	}

 out_unlock:
	mutex_unlock(&ctrl_data.lock);

 out:
	return ret;
}

static struct file_operations ach_ctrl_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = ach_ctrl_ioctl,
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
	mutex_init(&ctrl_data.lock);

	/* Create ctrl device */
	misc_register(&ach_misc_device);
	printk(KERN_INFO
	       "ach device has been registered with a max of %u devices\n",
	       max_devices);

	/* Common channel device stuff */

	/* We'll use own major number as we want to control the minor numbers ourself */
	{
		dev_t dev_num;

		ctrl_data.ach_ch_class = class_create(THIS_MODULE, "ach_ch");
		if (IS_ERR(ctrl_data.ach_ch_class)) {
			ret = PTR_ERR(ctrl_data.ach_ch_class);
			printk(KERN_ERR "ach: Failed to create class\n");
			goto out_deregister;
		}

		/* Allocate major */
		dev_num = 0;
		ret = alloc_chrdev_region(&dev_num, 0, max_devices, "ach_ch");
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
		printk(KERN_ERR "ach failed cleaning up\n");

	class_destroy(ctrl_data.ach_ch_class);
	ctrl_data.ach_ch_class = NULL;

	unregister_chrdev_region(MKDEV(ctrl_data.major, 0), max_devices);

	misc_deregister(&ach_misc_device);
	printk(KERN_INFO "ach device has been unregistered\n");
}

module_init(ach_init);
module_exit(ach_exit);

/* Local Variables:    */
/* mode: C             */
/* c-basic-offset: 8   */
/* indent-tabs-mode: t */
/* End:                */
