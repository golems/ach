;; Copyright (c) 2009, Georgia Tech Research Corporation
;; All rights reserved.
;;
;; Redistribution and use in source and binary forms, with or without
;; modification, are permitted provided that the following conditions
;; are met:
;;
;; * Redistributions of source code must retain the above copyright
;;   notice, this list of conditions and the following disclaimer.
;;
;; * Redistributions in binary form must reproduce the above copyright
;;   notice, this list of conditions and the following disclaimer in
;;   the documentation and/or other materials provided with the
;;   distribution.
;;
;; * Neither the name of the copyright holder(s) nor the names of its
;;   contributors may be used to endorse or promote products derived
;;   from this software without specific prior written permission.
;;
;; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
;; "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
;; LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
;; FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
;; COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
;; INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
;; (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
;; SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
;; HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
;; STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
;; ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
;; OF THE POSSIBILITY OF SUCH DAMAGE.


;; Author: Neil T. Dantam



(defpackage :ach
  (:use :cl :binio))

(in-package :ach)

(defstruct channel
  direction
  process)

;; let's make the delimiters int32s.  That'll be faster, right?

(defparameter *size-delim* (decode-uint (map 'vector #'char-code "size") :little))
(defparameter *data-delim* (decode-uint (map 'vector #'char-code "data") :little))

(defun channel-input (channel)
  (sb-ext:process-input (channel-process channel)))

(defun channel-output (channel)
  (sb-ext:process-output (channel-process channel)))

(defun ach-open (channel-name &key (direction :input))
  (let ((proc-input (if (eq direction :output) :stream))
        (proc-output (if (eq direction :input) :stream)))
    (make-channel
     :direction direction
     :process (sb-ext:run-program "achpipe.bin"
                                  (list (case direction
                                          (:input "-s")
                                          (:output "-p")
                                          (otherwise (error "Invalid direction: ~A"
                                                            direction)))
                                        channel-name)
                                  :search t
                                  :wait nil
                                  :input proc-input
                                  :input-element-type '(unsigned-byte 8)
                                  :output proc-output
                                  :output-element-type '(unsigned-byte 8)
                                  :error *standard-output*))))


;; Submitted a patch to SBCL on 20090827 to support binary io to
;; run-program.  Let's workaround until it filters through.

(defun read-bytes-dammit (buffer stream)
  (declare (type (array unsigned-byte) buffer))
  (if (eq (stream-element-type stream) 'unsigned-byte)
      (read-sequence buffer stream)
      (dotimes (i (length buffer))
        (setf (aref buffer i)
              (read-byte stream)))))

(defun write-bytes-dammit (buffer stream)
  (declare (type (array unsigned-byte) buffer))
  (if (eq (stream-element-type stream) 'unsigned-byte)
      (write-sequence buffer stream)
      (dotimes (i (length buffer))
        (write-byte (aref buffer i) stream ))))

(defun ach-read (channel)
  (assert (eq :input (channel-direction channel)) ()
          "Channel direction must be :INPUT, not ~S"
          (channel-direction channel))
  (let ((s (channel-output channel))
        (size-buf (make-array 12 :element-type 'unsigned-byte)))
    (read-bytes-dammit size-buf s)
    (assert (= *size-delim*
               (decode-uint size-buf :little 0)) ()
               "Invalid size delimiter: ~A" (subseq size-buf 0 4))
    (assert (= *data-delim*
               (decode-uint size-buf :little 8)) ()
               "Invalid size delimiter: ~A" (subseq size-buf 8))
    (let* ((size (decode-uint size-buf :big 4))
           (buffer (make-array size :element-type '(unsigned-byte 8))))
      (princ size)
      ;(read-bytes-dammit buffer s)
      (read-sequence buffer s)
      buffer)))



(defun ach-write (channel buffer)
  (declare (type (array unsigned-byte) buffer))
  (assert (eq :output (channel-direction channel)) ()
          "Channel direction must be :OUTPUT, not ~S"
          (channel-direction channel))
  (let ((s (channel-input channel))
        (size-buf (make-array 12 :element-type 'unsigned-byte)))
    (encode-int *size-delim* :little 0)
    (encode-int (length buffer) :big 4)
    (encode-int *data-delim* :little 8)
    (write-bytes-dammit size-buf s)
    (write-bytes-dammit buffer s)))



(defun ach-close (channel)
  (sb-ext:process-kill (channel-process channel) sb-posix:sigterm)
  (setf (channel-direction channel) nil)
  channel)


