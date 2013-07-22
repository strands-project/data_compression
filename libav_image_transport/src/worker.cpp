/*
 *    worker.cpp
 *
 *    Copyright 2013 Dominique Hunziker
 *
 *    Licensed under the Apache License, Version 2.0 (the "License");
 *    you may not use this file except in compliance with the License.
 *    You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 *    Unless required by applicable law or agreed to in writing, software
 *    distributed under the License is distributed on an "AS IS" BASIS,
 *    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *    See the License for the specific language governing permissions and
 *    limitations under the License.
 *
 *     \author/s: Dominique Hunziker
 */

#include "libav_image_transport/worker.hpp"

namespace libav_image_transport
{

Worker::Worker(void) :
		keep_running_(true), capacity_(100)
{
}

void Worker::resize(unsigned int capacity)
{
	assert(capacity > 0);
	capacity_ = capacity;
}

void Worker::start(void)
{
	thread_ = boost::thread(&Worker::run, this);
}

void Worker::stop(void)
{
	boost::mutex::scoped_lock lock(mutex_);

	keep_running_ = false;
	space_available_.notify_all();
	data_available_.notify_all();

	finished_.wait(lock);
}

void Worker::schedule(const boost::function<void()> &task)
{
	boost::mutex::scoped_lock lock(mutex_);

	while (keep_running_ && queue_.size() >= capacity_)
		space_available_.wait(lock);

	queue_.push(task);
	data_available_.notify_one();
}

void Worker::run(void)
{
	boost::function<void()> task;

	while (1)
	{
		{
			boost::mutex::scoped_lock lock(mutex_);

			while (keep_running_ && queue_.empty())
				data_available_.wait(lock);

			if (!keep_running_)
				break;

			task = queue_.front();
			queue_.pop();
			space_available_.notify_one();
		}

		task();
	}

	finished_.notify_all();
}

} /* namespace libav_image_transport */
