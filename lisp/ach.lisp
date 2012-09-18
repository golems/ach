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

;; TODO:
;; - Grovel ach_channel_t
;; - Grovel size_t
;; - Return enum types

(cffi:define-foreign-library libach
  (:unix "libach.so")
  (t (:default "libach")))

(cffi:use-foreign-library libach)

(cffi:defcvar "ach_channel_size" :uint)

(defstruct ach-handle
  pointer
  name
  opened)

(cffi:defcfun "ach_open" :int
  (chan :pointer)
  (name :string)
  (attr :pointer))

;; better to use CFFI groveler for this
#-CFFI-FEATURES:X86-64
(cffi:defctype size-t :unsigned-int)
#+CFFI-FEATURES:X86-64
(cffi:defctype size-t :uint64)

(cffi:defcfun "ach_get" :int
  (chan :pointer)
  (buf :pointer)
  (size size-t)
  (frame-size :pointer)
  (abstime :pointer)
  (ach-options :int))


(cffi:defcfun "ach_put" :int
  (chan :pointer)
  (buf :pointer)
  (len size-t))

(cffi:defcfun "ach_flush" :int
  (chan :pointer))

(cffi:defcfun "ach_close" :int
  (chan :pointer))

;; (cffi:defcfun "ach_chmod" :int
;;   (chan :pointer)
;;   (mode libc::mode-t))

(cffi:defcfun "ach_result_to_string" :string
  (r :int))

(cffi:defcfun "ach_create" :int
  (name :string)
  (frame-cnt size-t)
  (frame-size size-t)
  (attr :pointer))

(define-condition ach-status (error)
  ((message
    :initarg :message)
   (type
    :initarg :type)))

(defun status-keyword (code)
  (cffi:foreign-enum-keyword 'ach-status code))

(defun ach-status (type fmt &rest args)
  (error 'ach-status
        :message (apply #'format nil fmt args)
        :type type))

(defun check-status (code fmt &rest args)
  (let ((k (status-keyword code)))
    (unless (eq :ok k)
      (apply #'ach-status k fmt args))
    k))

(defmethod print-object ((object ach-status) stream)
  (print-unreadable-object (object stream :type t :identity t)
    (format stream "(~A): ~A"
            (slot-value object 'type)
            (slot-value object 'message))))

(defun create-channel (name &key
                       (frame-count 16)
                       (frame-size 1024))
  (let ((r (ach-create name frame-count frame-size (cffi:null-pointer))))
    (check-status r "creating channel")))

(defun close-channel (channel)
  (let ((pointer (ach-handle-pointer channel)))
    (assert pointer () "No channel pointer")
    (assert (ach-handle-opened channel) () "Channel already closed")
    (unwind-protect
         (progn
           (sb-ext:cancel-finalization channel)
           (let ((r (ach-close pointer)))
             (assert (zerop r) () "Couldn't close channel: ~A"
                     (ach-result-to-string r))
             (setf (ach-handle-opened channel) nil)))
      (cffi:foreign-free pointer)
      (setf (ach-handle-pointer channel) nil))
    nil))


(defun open-channel (name)
  "Open ach channel NAME.

Returns -- handle to the channel"
  (let ((handle (make-ach-handle :name name
                                 :opened nil
                                 :pointer nil))
        (ptr (cffi:foreign-alloc :int8 :count *ach-channel-size*)))
    (setf (ach-handle-pointer handle) ptr)
    (sb-ext:finalize handle
                     (lambda ()
                       ;(format t "Finalizing")
                       (close-channel (make-ach-handle
                                       :name name
                                       :opened t
                                       :pointer ptr))))
    (let ((r (ach-open (ach-handle-pointer handle)
                       name
                       (cffi-sys:null-pointer))))
      (assert (zerop r) () "Couldn't open channel: ~A"
              (ach-result-to-string r)))
    (setf (ach-handle-opened handle) t)
    handle))

(defun put-pointer (channel pointer length)
  "Put LENGTH octets at raw POINTER onto CHANNEL."
  (let ((r (ach-put (ach-handle-pointer channel)
                    pointer length)))
    (assert (zerop r) () "Couldn't write data: ~A"
            (ach-result-to-string r)))
  nil)

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

;; (defun chmod (channel mode)
;;   (declare (fixnum mode))
;;   (let ((r (ach-chmod (ach-handle-pointer channel) mode)))
;;     (assert (zerop r) () "Couldn't chmod to ~A: ~A"
;;             mode (ach-result-to-string r))))

(defun get-pointer (channel pointer length
                    &key wait last)
  "Get from CHANNEL into raw pointer POINTER.

Returns -- (values ach-status frame-size)."
  (let ((int-options (logior (if wait (cffi:foreign-enum-value 'ach-status :wait) 0)
                             (if last (cffi:foreign-enum-value 'ach-status :last) 0)))
        (frame-size)
        (r))
    (cffi:with-foreign-object (p-frame-size 'size-t)
      (setq r (ach-get (ach-handle-pointer channel)
                       pointer length p-frame-size
                       (cffi:null-pointer) int-options))
      (setq frame-size (cffi:mem-ref p-frame-size 'size-t)))
    (values (status-keyword r) frame-size )))

(defun get-buffer (channel &key count buffer wait last)
  "Get frame from CHANNEL.

If COUNT and BUFFER are unspecified, inspect the channel to determine
the size.  If BUFFER is given, get the data into BUFFER.  If COUNT is
given, create of buffer with COUNT octets.

Returns -- (values BUFFER ach-status)"
  (assert (and (ach-handle-opened channel)
               (ach-handle-pointer channel)) ()
               "Invalid channel: ~A" channel)
  (cond
    (buffer
     (check-type buffer (simple-array (unsigned-byte 8)))
     (let ((frame-size) (r-keyword))
       (cffi-sys:with-pointer-to-vector-data (pbuf buffer)
         (multiple-value-setq (r-keyword frame-size)
           (get-pointer channel pbuf (length buffer) :wait wait :last last)))
       (unless (or (eq :ok r-keyword) (eq :missed-frame r-keyword))
         (ach-status r-keyword "Error reading frame: ~A" r-keyword))
       ;;FIXME: handle this
       (assert (= frame-size (length buffer)) () "Mismatched frame size")
       (values buffer r-keyword)))
    (count
     (get-buffer channel
                 :buffer (make-array count :element-type '(unsigned-byte 8))
                 :wait wait
                 :last last))
    (t
     (get-buffer channel
                 :wait wait
                 :last last
                 :count (multiple-value-bind (r-keyword frame-size)
                            (get-pointer channel (cffi:null-pointer) 0
                                         :wait wait :last last)
                          (if (eq r-keyword :overflow)
                              frame-size
                              (ach-status r-keyword "Error getting size: ~A"
                                          r-keyword)))))))

(defun flush-channel (channel)
  "Flush all unseen frames on CHANNEL."
  (assert (and (ach-handle-opened channel)
               (ach-handle-pointer channel)) ()
               "Invalid channel: ~A" channel)
  (let ((r-kw (status-keyword (ach-flush (ach-handle-pointer channel)))))
    (unless (eq :ok r-kw)
      (ach-status r-kw "Unable to flush channel: ~A" r-kw))
    r-kw))
