;;;; -*- Lisp -*-
;;;;
;;;; Copyright (c) 2009-2012, Georgia Tech Research Corporation
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

;; Author: Neil T. Dantam


(in-package :ach)
;;;;;;;;;;;;
;;; CFFI ;;;
;;;;;;;;;;;;

(cffi:define-foreign-library libach
  (:unix "libach.so")
  (t (:default "libach")))

(cffi:use-foreign-library libach)

(defstruct ach-handle
  pointer
  name
  open)

(cffi:defcfun "ach_open" ach-status
  (chan :pointer)
  (name :string)
  (attr :pointer))

(cffi:defcfun "ach_get" ach-status
  (chan :pointer)
  (buf :pointer)
  (size size-t)
  (frame-size :pointer)
  (abstime :pointer)
  (ach-options :int))

(cffi:defcfun "ach_put" ach-status
  (chan :pointer)
  (buf :pointer)
  (len size-t))

(cffi:defcfun "ach_flush" ach-status
  (chan :pointer))

(cffi:defcfun "ach_close" ach-status
  (chan :pointer))

(cffi:defcfun "ach_chmod" ach-status
  (chan :pointer)
  (mode mode-t))

(cffi:defcfun "ach_create" ach-status
  (name :string)
  (frame-cnt size-t)
  (frame-size size-t)
  (attr :pointer))

(define-condition ach-status (error)
  ((message
    :initarg :message)
   (type
    :initarg :type)))

(defun ach-status (type fmt &rest args)
  (error 'ach-status
        :message (apply #'format nil fmt args)
        :type type))

(defun check-status (code fmt &rest args)
  (unless (eq :ok code)
    (apply #'ach-status code fmt args))
  code)

(defun check-channel-open (channel)
  (check-type channel ach-handle)
  (assert (ach-handle-open channel))
  (assert (and (ach-handle-pointer channel)
               (not (cffi:null-pointer-p (ach-handle-pointer channel)))))
  channel)

(defmethod print-object ((object ach-status) stream)
  (print-unreadable-object (object stream :type t :identity t)
    (format stream "(~A): ~A"
            (slot-value object 'type)
            (slot-value object 'message))))

(defun create-channel (name &key
                       (frame-count 16)
                       (frame-size 1024))
  (check-status (ach-create name frame-count frame-size (cffi:null-pointer))
                "creating channel"))

(defun close-channel (channel)
  "Close the channel."
  (check-type channel ach-handle)
  (let ((pointer (ach-handle-pointer channel)))
    (unwind-protect
         (progn
           (sb-ext:cancel-finalization channel)
           (when (ach-handle-open channel)
             (check-status (ach-close pointer)
                           "Couldn't close channel: ~A")
             (setf (ach-handle-open channel) nil)))
      (cffi:foreign-free pointer)
      (setf (ach-handle-pointer channel) nil))
    nil))

(defun open-channel (name)
  "Open ach channel NAME.

Also registers a finalizer to close the channel.

Returns -- handle to the channel"
  (let ((handle (make-ach-handle :name name
                                 :open nil
                                 :pointer nil))
        (ptr (cffi:foreign-alloc 'ach-channel-t)))
    (setf (ach-handle-pointer handle) ptr)
    (sb-ext:finalize handle
                     (lambda ()
                       (close-channel (make-ach-handle
                                       :name name
                                       :open t
                                       :pointer ptr))))
    (check-status (ach-open (ach-handle-pointer handle)
                            name
                            (cffi-sys:null-pointer))
                  "Couldn't open channel")
    (setf (ach-handle-open handle) t)
    handle))

(defun put-pointer (channel pointer length)
  "Put LENGTH octets at raw POINTER onto CHANNEL.

WARNING -- if POINTER or LENGTH are wrong, this may trash your system."
  (check-channel-open channel)
  (check-status (ach-put (ach-handle-pointer channel)
                         pointer length)
                "Couldn't put data"))

(defun put-buffer (channel buffer)
  "Put simple-array BUFFER onto CHANNEL."
  (let ((bytes
         (etypecase buffer
           ((or (simple-array (unsigned-byte 8))
                (simple-array (signed-byte 8)))
            1)
           ((or (simple-array (unsigned-byte 16))
                (simple-array (signed-byte 16)))
            2)
           ((or (simple-array (unsigned-byte 32))
                (simple-array (signed-byte 32))
                (simple-array float))
            4)
           ((or (simple-array (unsigned-byte 64))
                (simple-array (signed-byte 64))
                (simple-array double-float))
            8))))
    (cffi-sys:with-pointer-to-vector-data (pbuf buffer)
      (put-pointer channel pbuf (* bytes (length buffer))))))

(defgeneric put-object (channel object))

(defmethod put-object (channel (object string))
  (cffi:with-foreign-string ((pointer length) object)
    (put-pointer channel pointer length)))

(defun chmod-channel (channel mode)
  (check-channel-open channel)
  (check-status (ach-chmod (ach-handle-pointer channel) mode)
                "Couldn't chmod to ~A" mode))

(defun get-pointer (channel pointer length
                    &key wait last)
  "Get from CHANNEL into raw pointer POINTER.

WARNING -- if POINTER or LENGTH are wrong, this may trash your system.

Returns -- (values ach-status frame-size)."
  (check-channel-open channel)
  (let ((int-options (logior (if wait (cffi:foreign-enum-value 'ach-status :wait) 0)
                             (if last (cffi:foreign-enum-value 'ach-status :last) 0)))
        (frame-size)
        (r))
    (cffi:with-foreign-object (p-frame-size 'size-t)
      (setq r (ach-get (ach-handle-pointer channel)
                       pointer length p-frame-size
                       (cffi:null-pointer) int-options))
      (setq frame-size (cffi:mem-ref p-frame-size 'size-t)))
    (values r frame-size )))

(defun get-buffer (channel &key count buffer wait last)
  "Get frame from CHANNEL.

If COUNT and BUFFER are unspecified, inspect the channel to determine
the size.  If BUFFER is given, fill the data into BUFFER.  If that
would overflow, cons a new buffer.  If COUNT is given, create of
buffer with COUNT octets.

Returns -- (values buffer ach-status frame-size)"
  (cond
    (buffer
     (check-type buffer (simple-array (unsigned-byte 8)))
     (multiple-value-bind (r frame-size)
         (cffi-sys:with-pointer-to-vector-data (pbuf buffer)
           (get-pointer channel pbuf (length buffer) :wait wait :last last))
       (cond
         ((or (eq :ok r) (eq :missed-frame r))
          (values buffer r frame-size))
         ((eq :overflow r)
          (get-buffer channel :count frame-size :wait wait :last last))
         (t (ach-status r "Error reading frame: ~A" r)))))
    (count
     (get-buffer channel
                 :buffer (make-array count :element-type '(unsigned-byte 8))
                 :wait wait
                 :last last))
    (t
     (get-buffer channel
                 :wait wait
                 :last last
                 :count (multiple-value-bind (r frame-size)
                            (get-pointer channel (cffi:null-pointer) 0
                                         :wait wait :last last)
                          (if (eq r :overflow)
                              frame-size
                              (ach-status r  "Error getting size")))))))

(defun flush-channel (channel)
  "Flush all unseen frames on CHANNEL."
  (check-channel-open channel)
  (check-status (ach-flush (ach-handle-pointer channel))
                "Unable to flush channel"))
