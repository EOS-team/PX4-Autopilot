/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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

#include "SubscriptionCallback.hpp"

#include <containers/LockGuard.hpp>
#include <px4_time.h>
#if defined(__PX4_EVL4)
#include <containers/EvlLockGuard.hpp>
#include <evl/clock.h>
#include <evl/event.h>
#endif
namespace uORB
{

// Subscription with callback that schedules a WorkItem
template<typename T>
class SubscriptionBlocking : public SubscriptionCallback
{
public:
	/**
	 * Constructor
	 *
	 * @param meta The uORB metadata (usually from the ORB_ID() macro) for the topic.
	 * @param interval_us The requested maximum update interval in microseconds.
	 * @param instance The instance for multi sub.
	 */
	SubscriptionBlocking(const orb_metadata *meta, uint32_t interval_us = 0, uint8_t instance = 0) :
		SubscriptionCallback(meta, interval_us, instance)
	{
	}

	virtual ~SubscriptionBlocking()
	{
#if defined(__PX4_EVL4)
		evl_close_mutex(&_mutex);
		evl_close_event(&_cv);
#else
		pthread_mutex_destroy(&_mutex);
		pthread_cond_destroy(&_cv);
#endif
	}

	void call() override
	{
		// signal immediately if no interval, otherwise only if interval has elapsed
		if ((_interval_us == 0) || (hrt_elapsed_time(&_last_update) >= _interval_us)) {
#if defined(__PX4_EVL4)
			evl_signal_event(&_cv);
#else
			pthread_cond_signal(&_cv);
#endif
		}
	}

	/**
	 * Block until updated.
	 * @param timeout_us The requested timeout in microseconds, or 0 to wait indefinitely.
	 *
	 * @return true only if topic was updated
	 */
	bool updatedBlocking(uint32_t timeout_us = 0)
	{
		if (!_registered) {
			registerCallback();
		}

		if (updated()) {
			// return immediately if updated
			return true;

		} else {
			// otherwise wait

			EvlLockGuard lg{_mutex};

			if (timeout_us == 0) {
				// wait with no timeout
#if defined(__PX4_EVL4)
				if (evl_wait_event(&_cv, &_mutex) == 0) {
					return updated();
				}

#else

				if (pthread_cond_wait(&_cv, &_mutex) == 0) {
					return updated();
				}

#endif

			} else {
				// otherwise wait with timeout based on interval

				// Calculate an absolute time in the future
				struct timespec ts;
#if not defined(__PX4_EVL4)
				px4_clock_gettime(CLOCK_REALTIME, &ts);
#else
				evl_read_clock(EVL_CLOCK_REALTIME, &ts);
#endif
				uint64_t nsecs = ts.tv_nsec + (timeout_us * 1000);
				static constexpr unsigned billion = (1000 * 1000 * 1000);
				ts.tv_sec += nsecs / billion;
				nsecs -= (nsecs / billion) * billion;
				ts.tv_nsec = nsecs;
#if not defined(__PX4_EVL4)
				if (px4_pthread_cond_timedwait(&_cv, &_mutex, &ts) == 0) {
					return updated();
				}
#else
				if (evl_timedwait_event(&_cv, &_mutex, &ts) == 0) {
					return updated();
				}
#endif
			}
		}

		return false;
	}

	/**
	 * Copy the struct if updated.
	 * @param data The data reference where the struct will be copied.
	 * @param timeout_us The requested timeout in microseconds, or 0 to wait indefinitely.
	 *
	 * @return true only if topic was updated and copied successfully.
	 */
	bool updateBlocking(T &data, uint32_t timeout_us = 0)
	{
		if (updatedBlocking(timeout_us)) {
			return copy(&data);
		}

		return false;
	}

private:

#ifdef __PX4_EVL4
	struct evl_mutex _mutex = EVL_MUTEX_INITIALIZER(nullptr, EVL_CLOCK_MONOTONIC, 0, EVL_MUTEX_NORMAL);
	struct evl_event _cv = EVL_EVENT_INITIALIZER(nullptr, EVL_CLOCK_REALTIME, EVL_CLONE_PRIVATE);
#else
	pthread_mutex_t _mutex = PTHREAD_MUTEX_INITIALIZER;
	pthread_cond_t	_cv = PTHREAD_COND_INITIALIZER;
#endif

};

} // namespace uORB
