;;;; -*- Lisp -*-
;;;;
;;;; Copyright (c) 2011, Georgia Tech Research Corporation
;;;; All rights reserved.
;;;;
;;;; Author(s): Neil T. Dantam <ntd@gatech.edu>
;;;; Georgia Tech Humanoid Robotics Lab
;;;; Under Direction of Prof. Mike Stilman
;;;;
;;;;
;;;; This file is provided under the following "BSD-style" License:
;;;;
;;;;
;;;;   Redistribution and use in source and binary forms, with or
;;;;   without modification, are permitted provided that the following
;;;;   conditions are met:
;;;;
;;;;   * Redistributions of source code must retain the above copyright
;;;;     notice, this list of conditions and the following disclaimer.
;;;;
;;;;   * Redistributions in binary form must reproduce the above
;;;;     copyright notice, this list of conditions and the following
;;;;     disclaimer in the documentation and/or other materials provided
;;;;     with the distribution.
;;;;
;;;;   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
;;;;   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
;;;;   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
;;;;   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
;;;;   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
;;;;   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
;;;;   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
;;;;   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
;;;;   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
;;;;   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
;;;;   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
;;;;   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
;;;;   POSSIBILITY OF SUCH DAMAGE.

(progn
  (in-package :ach)
  (cc-flags "--std=gnu99")
  (include "stdint.h")
  (include "pthread.h")
  (include "stdlib.h")
  (include "stdio.h")
  (include "unistd.h")
  (include "time.h")
  (include "sys/stat.h")
  (include "ach.h")
  (ctype mode-t "mode_t")
  (ctype size-t "size_t")
  (cstruct ach-channel-t "ach_channel_t")
  (cenum (ach-status :define-constants nil)
         ((:ok "ACH_OK"))
         ((:overflow "ACH_OVERFLOW"))
         ((:invalid-name "ACH_INVALID_NAME"))
         ((:bad-shm-file "ACH_BAD_SHM_FILE"))
         ((:failed-syscall "ACH_FAILED_SYSCALL"))
         ((:stale-frames "ACH_STALE_FRAMES"))
         ((:missed-frame "ACH_MISSED_FRAME"))
         ((:timeout "ACH_TIMEOUT"))
         ((:eexist "ACH_EEXIST"))
         ((:enoent "ACH_ENOENT"))
         ((:closed "ACH_CLOSED"))
         ((:einval "ACH_EINVAL"))
         ((:bug "ACH_BUG")))
  (cenum (ach-get-opts :define-constants nil)
         ((:wait "ACH_O_WAIT"))
         ((:last "ACH_O_LAST"))
         ((:copy "ACH_O_COPY"))))
