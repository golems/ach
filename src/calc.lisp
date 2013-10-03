;;;; -*- Lisp -*-
;;;;
;;;; Copyright (c) 2013, Georgia Tech Research Corporation
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


(defvar *data-string*)
(defvar *data-lines*)
(defvar *data-array*)
(defvar *data-sorted*)

(cffi:defcfun atof :double
  (str :string))

(defun read-dat (file-name)
  (with-open-file (s file-name :direction :input)
    (let ((buf (make-string (file-length s))))
      (read-sequence buf s)
      buf)))

(defun split-lines (str)
  (loop
     for start = 0 then (1+ end)
     for end = (position #\Newline str :start start)
     while end
     collect (subseq str start end)))

(defun parse-strings (lines)
  (let ((x (make-array (length lines) :element-type 'double-float))
        (i 0))
    (declare (fixnum i))
    (dolist (str lines)
      (setf (aref x i) (atof str))
      (incf i))
    x))

(defun median (data)
  (let* ((n (length data))
         (n/2 (truncate (/ n 2))))
    (if (oddp n)
        (elt data  n/2)
        (/ (+ (elt data n/2)
              (elt data (1- n/2)))
           2))))

(defun mean (data)
  (/ (reduce #'+ data)
     (length data)))

(defun nine-frac (n)
  (loop
     for i from 1 upto n
     for nine = (/ 9d0 (expt 10 i))
     for x = nine then (+ x nine)
     finally (return x)))

(defun nines-value (n-nines data)
  (let* ((frac (nine-frac n-nines))
         (n (length data))
         (i (round (* frac (1- n)))))
    (aref data i)))

(defun analyze (file-name)
  (let* ((data (sort (parse-strings (split-lines (read-dat file-name)))
                     #'<))
         (n (length data)))
    (let ((min (aref data 0))
          (max (aref data (1- n)))
          (median (median data))
          (mean (mean data))
          (nine-2 (nines-value 2 data))
          (nine-3 (nines-value 3 data))
          (nine-4 (nines-value 4 data))
          (nine-5 (nines-value 5 data))
          (nine-6 (nines-value 6 data))
          (nine-7 (nines-value 6 data))
          (nine-8 (nines-value 6 data))
          (nine-9 (nines-value 6 data)))
      (format t "~&~,3F & ~,3F & ~,3F & ~,3F & ~,3F & ~,3F & ~,3F & ~,3F~&"
              min median mean
              nine-2 nine-3 nine-4 nine-5
              max)
      (list
       :min min
       :max max
       :median median
       :mean mean
       :nine-2 nine-2
       :nine-3 nine-3
       :nine-4 nine-4
       :nine-5 nine-5
       :nine-6 nine-6
       :nine-7 nine-7
       :nine-8 nine-8
       :nine-9 nine-9))))


(defun cmp-sloc (file)
  (let ((data (sort (with-open-file (s file)
                      (loop for x = (read s nil nil)
                         while x
                         collect (list (string (first x))
                                       (second x))))
                    (lambda (a b)
                      (< (second a) (second b))))))
    (let ((min (apply #'min (map 'list #'second data))))
      (loop for (sym x) in data
         do
           (format t "~&~:(~A~) & ~A & ~,2Fx \\\\~&" sym x (/ x min))))))
