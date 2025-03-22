/****************************************************************************
 *
 *   Copyright (C) 2019 PX4 Development Team. All rights reserved.
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

#include <pthread.h>

#if defined(__PX4_EVL4)
#include <px4_platform_common/evl_helper.h>
#include <evl/mutex.h>
#endif

class LockGuard
{
public:
#if defined(__PX4_EVL4)
	explicit LockGuard(struct evl_mutex &mutex) :
		_mutex(mutex)
	{
		int eret;
		__Tcall_assert(eret, evl_lock_mutex(&_mutex));
	}
#else
	explicit LockGuard(pthread_mutex_t &mutex) :
		_mutex(mutex)
	{
		pthread_mutex_lock(&_mutex);
	}
#endif

	LockGuard(const LockGuard &other) = delete;
	LockGuard &operator=(const LockGuard &other) = delete;

	~LockGuard()
	{
#if defined(__PX4_EVL4)
		int eret;
		__Tcall_assert(eret, evl_unlock_mutex(&_mutex));
#else
		pthread_mutex_unlock(&_mutex);
#endif
	}

private:

#if defined(__PX4_EVL4)
	struct evl_mutex &_mutex;
#else
	pthread_mutex_t &_mutex;
#endif

};
