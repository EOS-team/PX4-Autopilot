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

/**
 * @file BlockingList.hpp
 *
 * A blocking intrusive linked list.
 */

#pragma once

#include "IntrusiveSortedList.hpp"
#include "LockGuard.hpp"

#include <pthread.h>
#include <stdlib.h>

#if defined(__PX4_EVL4)
#include <evl/mutex.h>
#include <evl/event.h>
#endif

template<class T>
class BlockingList : public IntrusiveSortedList<T>
{
public:

	~BlockingList()
	{
#if defined(__PX4_EVL4)
		evl_close_mutex(&_mutex);
		evl_close_event(&_cv);
#else
		pthread_mutex_destroy(&_mutex);
		pthread_cond_destroy(&_cv);
#endif
	}

	void add(T newNode)
	{
		LockGuard lg{_mutex};
		IntrusiveSortedList<T>::add(newNode);
	}

	bool remove(T removeNode)
	{
		LockGuard lg{_mutex};
		return IntrusiveSortedList<T>::remove(removeNode);
	}

	size_t size()
	{
		LockGuard lg{_mutex};
		return IntrusiveSortedList<T>::size();
	}

	void clear()
	{
		LockGuard lg{_mutex};
		IntrusiveSortedList<T>::clear();
	}

#if defined(__PX4_EVL4)
	struct evl_mutex &mutex() { return _mutex; }
#else
	pthread_mutex_t &mutex() { return _mutex; }
#endif

private:
#if defined(__PX4_EVL4)
	struct evl_mutex _mutex = EVL_MUTEX_INITIALIZER(nullptr, EVL_CLOCK_MONOTONIC, 0, EVL_MUTEX_NORMAL);
	struct evl_event _cv = EVL_EVENT_INITIALIZER(nullptr, EVL_CLOCK_MONOTONIC, EVL_CLONE_PRIVATE);
#else
	pthread_mutex_t	_mutex = PTHREAD_MUTEX_INITIALIZER;
	pthread_cond_t	_cv = PTHREAD_COND_INITIALIZER;
#endif

};
