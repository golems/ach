%module ach
%{

#include <stdint.h>
#include "ach.h"        

%}

%typemap(cin) uint64_t ":uint64"
%typemap(cin) size_t ":uint32"
%typemap(cin) uint32_t ":uint32"

%include "ach.h"
