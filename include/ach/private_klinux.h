/* -*- mode: C; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2008-2014, Georgia Tech Research Corporation
 * Copyright (c) 2013-2014, Prevas A/S
 * Copyright (c) 2015, Rice University
 * All rights reserved.
 *
 * Author(s): Neil T. Dantam <ntd@rice.edu>
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
 *   * Neither the name of the copyright holder the names of its
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

/** \file ach_private_linux.h
 *  \author Neil T. Dantam
 */

#ifndef ACH_PRIVATE_KLINUX_H
#define ACH_PRIVATE_KLINUX_H

#define ACH_KLINUX

#include "private_generic.h"

static int
get_errno(enum ach_status r) {
    switch(r) {
    case ACH_OK:              return 0;
    case ACH_OVERFLOW:        return EMSGSIZE;
    case ACH_INVALID_NAME:    return ENAMETOOLONG;
    case ACH_BAD_SHM_FILE:    return EBADSLT;
    case ACH_FAILED_SYSCALL:  return EIO;
    case ACH_EAGAIN:          return EAGAIN;
    case ACH_MISSED_FRAME:    return EREMOTEIO;
    case ACH_TIMEOUT:         return ETIME;
    case ACH_EEXIST:          return EEXIST;
    case ACH_ENOENT:          return ENOENT;
    case ACH_CLOSED:          return ESHUTDOWN;
    case ACH_BUG:             return EPERM;
    case ACH_EINVAL:          return EINVAL;
    case ACH_CORRUPT:         return EUCLEAN;
    case ACH_BAD_HEADER:      return EPROTO;
    case ACH_EACCES:          return EACCES;
    case ACH_CANCELED:        return ECANCELED;
    case ACH_EFAULT:          return EFAULT;
    case ACH_EINTR:           return EINTR;
    case ACH_ENOTSUP:         return ENOTSUPP;
    }
    return get_errno(ACH_BUG);
}

#endif /* ACH_PRIVATE_KLINUX_H */

/* Local Variables:    */
/* mode: C             */
/* c-basic-offset: 8   */
/* indent-tabs-mode: t */
/* End:                */
