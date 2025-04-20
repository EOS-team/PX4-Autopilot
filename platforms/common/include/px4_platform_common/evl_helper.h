/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
#pragma once

#include <string.h>
#include <stdlib.h>
#include <evl/proxy.h>
#include <evl/sched.h>
#include <evl/thread.h>

#define O_OOB 010000000000

#define __stringify_1(x...)	#x
#define __stringify(x...)	__stringify_1(x)
#define evl_warn_failed(__fmt, __args...)			\
	evl_eprintf("%s:%d: FAILED: " __fmt "\n",	\
		    __FILE__, __LINE__, ##__args)

#define __Tcall(__ret, __call)				\
	({						\
		(__ret) = (__call);			\
		if (__ret < 0) {			\
			evl_warn_failed("%s (=%s)",		\
					__stringify(__call),	\
					strerror(-(__ret)));	\
		}					\
		(__ret) >= 0;				\
	})

#define __Tcall_assert(__ret, __call)		\
	do {					\
		if (!__Tcall(__ret, __call))	\
			exit(__ret);	\
	} while (0)

// Attach to evl core and set thread name, scheduling policy and priority.
#define __attach_and_setsched(__policy, __prio, __fmt, __args...)		\
	do {								\
		int __ret;						\
		__Tcall_assert(__ret, evl_attach_self(__fmt, ##__args));	\
		struct evl_sched_attrs __attrs;				\
		__attrs.sched_policy = __policy;				\
		__attrs.sched_priority = __prio;				\
		__Tcall_assert(__ret, evl_set_schedattr(__ret, &__attrs));	\
	} while (0)
