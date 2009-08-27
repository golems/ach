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

(defpackage :binio
  (:use :cl)
  (:export :decode-uint :decode-sint :encode-int))

;; types u?int{8,16,32,63}, double, float


(in-package :binio)

(defun index-endian (index start count endian)
  (declare (fixnum start count index))
  (case endian
    (:little
     (+ start index))
    (:big
     (+ start count -1 (- index)))
    (otherwise
     (error "endian must be :big or :little, not ~S" endian))))

(defun aref-endian (buffer index start count endian)
  (declare (fixnum start count index))
  (aref buffer (index-endian index start count endian)))

(defun (setf aref-endian) (value buffer index start count endian)
  (declare (fixnum start count index))
  (setf (aref buffer (index-endian index start count endian))
        value))



;; fun : (lambda (elt little-endian-index) ...)
;;(defun map-endian-wise (fun buffer start count endian)
;;  (declare (fixnum start count))
;;  (declare (type symbol endian))
;;  (assert (or (eq endian :big) (eq endian :little)) ()
;;          "endian must be :big or :little, not ~S" endian)
;;  (let ((i0 (case endian
;;              (:little start)
;;              (:big (+ start count -1))
;;              (otherwise 0)))
;;        (delta (case endian
;;                 (:little 1)
;;                 (:big -1)
;;                 (otherwise 0)))
;;        (i-end (cond  ;; sbcl warns on (case...) here, weird...
;;                 ((eq endian :little) (+ start count))
;;                 ((eq endian :big) (1- start))
;;                 (t 0))))
;;    (do ((i i0 (+ i delta))
;;         (j 0 (1+ j)))
;;        ((= i i-end))
;;      (funcall fun (aref buffer i) j))))


(defun decode-uint (buffer endian &optional (start 0) (bits 32))
  (declare (fixnum start bits)
           (type (array unsigned-byte) buffer))
  (let ((accum 0)
        (count (/ bits 8)))
    (declare (integer accum))
    (dotimes (i count)
      (setf (ldb (byte 8 (* 8 i)) accum)
            (aref-endian buffer i start count endian)))
    accum))


(defun decode-sint (buffer endian &optional (start 0) (bits 32) )
  (declare (fixnum start bits)
           (type (array unsigned-byte) buffer))
  (let ((result (decode-uint buffer endian start bits))
        (count (/ bits 8)))
    (when (= (ldb (byte 1 (1- (* 8 count))) result) 1) ; sign bit, negative
      (decf result (ash 1 (* 8 count))))
    result))

(defun encode-int (val endian &optional (start 0) (bits 32))
  (declare (integer val)
           (fixnum start bits)
           (symbol endian))
  (let* ((count (/ bits 8))
         (buffer (make-array count
                             :element-type 'unsigned-byte)))
    (dotimes (i count)
      (setf (aref-endian buffer i start count endian)
            (ldb (byte 8 (* i 8)) val)))
    buffer))

