/* -*- mode: C; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2013-2014, Georgia Tech Research Corporation
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


#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <Python.h>

#include <time.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <assert.h>
#include <string.h>
#include <stdio.h>
#include <pthread.h>
#include <unistd.h>
#include <ctype.h>
#include <sys/stat.h>

#include <string.h>
#include <inttypes.h>

#include "ach.h"

PyMODINIT_FUNC initach_py(void);

static PyObject *ach_py_error;

static ach_channel_t *parse_channel_pointer( PyObject *i ) {
    if( PyInt_Check(i) ) {
        return (ach_channel_t*)PyInt_AsLong(i);
    } else if ( PyLong_Check(i) ) {
        return (ach_channel_t*)PyLong_AsVoidPtr(i);
    } else {
        PyErr_SetString( PyExc_TypeError, "invalid channel pointer" );
        return NULL;
    }
}

static PyObject  *raise_error( ach_status_t r ) {
    PyErr_SetString( ach_py_error, ach_result_to_string(r) );
    return NULL;
}

static PyObject *
ach_error( PyObject *self, PyObject *args ) {
    (void)self;
    int r;
    if( !PyArg_ParseTuple(args, "i", &r ) ) {
        return NULL;
    }
    return raise_error((ach_status_t)r);
}

static PyObject *
open_channel( PyObject *self, PyObject *args ) {
    (void)self;
    const char *name = NULL;
    long frame_count = 0, frame_size = 0;
    if( !PyArg_ParseTuple(args, "sll", &name, &frame_count, &frame_size ) ) {
        return NULL;
    }
    /* Alloc struct */
    ach_channel_t *c = (ach_channel_t*)malloc(sizeof(ach_channel_t));

    /* Open it */
    ach_status_t r = ach_open(c, name, NULL);

    /* Create channel if necessary */
    if( ACH_ENOENT == r ) {
        r = ach_create( name, (size_t)frame_count, (size_t)frame_size, NULL );
        if( ach_status_match(r, ACH_MASK_OK | ACH_MASK_EEXIST) ) {
            r = ach_open(c, name, NULL);
        }
    }

    /* Check result */
    if( ACH_OK != r ) {
        return raise_error(r);
    }

    return PyLong_FromVoidPtr(c);
}

static PyObject *
close_channel( PyObject *self, PyObject *args ) {
    (void)self;

    PyObject *py_chan;
    if( !PyArg_ParseTuple(args, "O", &py_chan) ) {
        return NULL;
    }

    ach_channel_t *c = parse_channel_pointer(py_chan);
    if( NULL == c ) {
        return NULL;
    }

    ach_status_t r = ach_close(c);
    if( ACH_OK != r ) {
        return raise_error(r);
    }

    free(c);

    Py_RETURN_NONE;
}

static PyObject *
result_string( PyObject *self, PyObject *args ) {
    (void)self;
    int r;
    if( !PyArg_ParseTuple(args, "i", &r ) ) {
        return NULL;
    }
    return PyString_FromString( ach_result_to_string((enum ach_status)r) );
}

static PyObject *
put_buf( PyObject *self, PyObject *args ) {
    (void)self;

    PyObject *py_chan, *b;
    // get arg objects
    if( !PyArg_ParseTuple(args, "OO", &py_chan, &b) ) {
        return NULL;
    }

    // parse channel
    ach_channel_t *c = parse_channel_pointer(py_chan);
    if( NULL == c ) {
        return NULL;
    }

    // parse buffer
    if( ! PyObject_CheckBuffer(b) ) {
        PyErr_SetString( PyExc_TypeError, "invalid buffer" );
        return NULL;
    }

    // view buffer
    Py_buffer buf;
    if( PyObject_GetBuffer( b, &buf, PyBUF_SIMPLE ) ) {
        PyErr_SetString( PyExc_BufferError, "couldn't view buffer" );
        return NULL;
    }

    // make the damn call
    ach_status_t r = ach_put( c, buf.buf, (size_t)buf.len );

    // check the result
    if( ACH_OK != r ) {
        PyErr_SetString( ach_py_error, ach_result_to_string(r) );
        return NULL;
    }

    // cleanup
    PyBuffer_Release(&buf);
    Py_RETURN_NONE;
}

static PyObject *
get_buf( PyObject *self, PyObject *args ) {
    (void)self;

    PyObject *py_chan, *b;
    int wait, last;
    // get arg objects
    if( !PyArg_ParseTuple(args, "OOii", &py_chan, &b, &wait, &last) ) {
        return NULL;
    }

    // parse channel
    ach_channel_t *c = parse_channel_pointer(py_chan);
    if( NULL == c ) {
        return NULL;
    }

    // parse buffer
    if( ! PyObject_CheckBuffer(b) ) {
        PyErr_SetString( PyExc_TypeError, "invalid buffer" );
        return NULL;
    }

    // view buffer
    Py_buffer buf;
    if( PyObject_GetBuffer( b, &buf, PyBUF_WRITABLE ) ) {
        PyErr_SetString( PyExc_BufferError, "couldn't view writable buffer" );
        return NULL;
    }

    // parse opts
    int opts = 0;
    if (wait) opts |= ACH_O_WAIT;
    if (last) opts |= ACH_O_LAST;

    // make the damn call
    size_t frame_size;
    ach_status_t r = ach_get( c, buf.buf, (size_t)buf.len,
                              &frame_size, NULL, opts );
    // cleanup
    PyBuffer_Release(&buf);

    // return
    switch(r) {
    case ACH_OK:
    case ACH_STALE_FRAMES:
    case ACH_MISSED_FRAME:
    case ACH_TIMEOUT:
    case ACH_CANCELED:
        return Py_BuildValue("ik", r, frame_size);

    case ACH_OVERFLOW:
    case ACH_INVALID_NAME:
    case ACH_BAD_SHM_FILE:
    case ACH_FAILED_SYSCALL:
    case ACH_CLOSED:
    case ACH_EEXIST:
    case ACH_ENOENT:
    case ACH_BUG:
    case ACH_EINVAL:
    case ACH_CORRUPT:
    case ACH_BAD_HEADER:
    case ACH_EACCES:
    case ACH_EINTR:
    case ACH_EFAULT:
    case ACH_ENOTSUP:
        return raise_error(r);
    }

    return NULL;

}

static PyObject *
flush_channel( PyObject *self, PyObject *args ) {
    (void)self;

    PyObject *py_chan;
    // get arg objects
    if( !PyArg_ParseTuple(args, "O", &py_chan) ) {
        return NULL;
    }

    // parse channel
    ach_channel_t *c = parse_channel_pointer(py_chan);
    if( NULL == c ) {
        return NULL;
    }

    // make the damn call
    ach_status_t r = ach_flush( c );

    // check the result
    if( ACH_OK != r ) {
        PyErr_SetString( ach_py_error, ach_result_to_string(r) );
        return NULL;
    }

    // cleanup
    Py_RETURN_NONE;
}

static PyObject *
chmod_channel( PyObject *self, PyObject *args ) {
    (void)self;

    PyObject *py_chan;
    int mode;
    // get arg objects
    if( !PyArg_ParseTuple(args, "Oi", &py_chan, &mode) ) {
        return NULL;
    }

    // parse channel
    ach_channel_t *c = parse_channel_pointer(py_chan);
    if( NULL == c ) {
        return NULL;
    }

    // make the damn call
    ach_status_t r = ach_chmod( c, (mode_t)mode );

    // check the result
    if( ACH_OK != r ) {
        PyErr_SetString( ach_py_error, ach_result_to_string(r) );
        return NULL;
    }

    // cleanup
    Py_RETURN_NONE;
}

static PyObject *
unlink_channel( PyObject *self, PyObject *args ) {
    (void)self;

    const char *name;
    // get arg objects
    if( !PyArg_ParseTuple(args, "s", &name) )  {
        return NULL;
    }

    // make the damn call
    ach_status_t r = ach_unlink( name );

    // check the result
    if( ACH_OK != r ) {
        PyErr_SetString( ach_py_error, ach_result_to_string(r) );
        return NULL;
    }

    // cleanup
    Py_RETURN_NONE;
}


static PyMethodDef module_methods[] = {
   { "open_channel", (PyCFunction)open_channel, METH_VARARGS, NULL },
   { "close_channel", (PyCFunction)close_channel, METH_VARARGS, NULL },
   { "put_buf", (PyCFunction)put_buf, METH_VARARGS, NULL },
   { "get_buf", (PyCFunction)get_buf, METH_VARARGS, NULL },
   { "ach_error", (PyCFunction)ach_error, METH_VARARGS, NULL },
   { "result_string", (PyCFunction)result_string, METH_VARARGS, NULL },
   { "flush_channel", (PyCFunction)flush_channel, METH_VARARGS, NULL },
   { "chmod_channel", (PyCFunction)chmod_channel, METH_VARARGS, NULL },
   { "unlink_channel", (PyCFunction)unlink_channel, METH_VARARGS, NULL },
   { NULL, NULL, 0, NULL }
};


PyMODINIT_FUNC initach_py() {
    PyObject *m;

    // methods
    m = Py_InitModule3("ach_py", module_methods, "Python extension module for the Ach IPC Library");
    if( NULL == m ) {
        return;
    }

    // error object
    static char errname[] =  "ach_py.AchException";
    ach_py_error = PyErr_NewException( errname, NULL, NULL);
    Py_INCREF( ach_py_error ); // Reference counts?  Get with the program python!
    PyModule_AddObject(m, "AchException", ach_py_error);

    // keyword/const objects
    PyModule_AddObject( m, "ACH_OK",               PyInt_FromLong( ACH_OK ) );
    PyModule_AddObject( m, "ACH_OVERFLOW",         PyInt_FromLong( ACH_OVERFLOW ) );
    PyModule_AddObject( m, "ACH_INVALID_NAME",     PyInt_FromLong( ACH_INVALID_NAME ) );
    PyModule_AddObject( m, "ACH_BAD_SHM_FILE",     PyInt_FromLong( ACH_BAD_SHM_FILE ) );
    PyModule_AddObject( m, "ACH_FAILED_SYSCALL",   PyInt_FromLong( ACH_FAILED_SYSCALL ) );
    PyModule_AddObject( m, "ACH_STALE_FRAMES",     PyInt_FromLong( ACH_STALE_FRAMES ) );
    PyModule_AddObject( m, "ACH_EAGAIN"      ,     PyInt_FromLong( ACH_EAGAIN ) );
    PyModule_AddObject( m, "ACH_LOCKED"      ,     PyInt_FromLong( ACH_LOCKED ) );
    PyModule_AddObject( m, "ACH_MISSED_FRAME",     PyInt_FromLong( ACH_MISSED_FRAME ) );
    PyModule_AddObject( m, "ACH_TIMEOUT",          PyInt_FromLong( ACH_TIMEOUT ) );
    PyModule_AddObject( m, "ACH_EEXIST",           PyInt_FromLong( ACH_EEXIST ) );
    PyModule_AddObject( m, "ACH_ENOENT",           PyInt_FromLong( ACH_ENOENT ) );
    PyModule_AddObject( m, "ACH_CLOSED",           PyInt_FromLong( ACH_CLOSED ) );
    PyModule_AddObject( m, "ACH_BUG",              PyInt_FromLong( ACH_BUG ) );
    PyModule_AddObject( m, "ACH_EINVAL",           PyInt_FromLong( ACH_EINVAL ) );
    PyModule_AddObject( m, "ACH_CORRUPT",          PyInt_FromLong( ACH_CORRUPT ) );
    PyModule_AddObject( m, "ACH_CANCELED",         PyInt_FromLong( ACH_CANCELED ) );
    PyModule_AddObject( m, "ACH_BAD_HEADER",       PyInt_FromLong( ACH_BAD_HEADER ) );
    PyModule_AddObject( m, "ACH_EACCES",           PyInt_FromLong( ACH_EACCES ) );
    PyModule_AddObject( m, "ACH_O_WAIT",           PyInt_FromLong( ACH_O_WAIT ) );
    PyModule_AddObject( m, "ACH_O_LAST",           PyInt_FromLong( ACH_O_LAST ) );
    /* PyModule_AddObject( m, "ACH_DEFAULT_FRAME_SIZE",   PyInt_FromLong( ACH_DEFAULT_FRAME_SIZE ) ); */
    /* PyModule_AddObject( m, "ACH_DEFAULT_FRAME_COUNT",  PyInt_FromLong( ACH_DEFAULT_FRAME_COUNT ) ); */
}
