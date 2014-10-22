/*
 *    worker.hpp
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

#ifndef LIBAV_IMAGE_TRANSPORT__WORKER_HPP_
#define LIBAV_IMAGE_TRANSPORT__WORKER_HPP_

#include <queue>
#include <boost/thread.hpp>
#include <boost/make_shared.hpp>

namespace libav_image_transport
{

class Worker
{
public:
	Worker(void);

	void resize(unsigned int capacity);

	void start(void);
	void stop(void);

	void schedule(const boost::function<void()> &task);

private:
	void run(void);

	boost::thread thread_;
	bool keep_running_;

	// Queue
	std::queue<boost::function<void()> > queue_;

	// Limit of queue size
	unsigned int capacity_;

	// Queue access lock
	boost::mutex mutex_;

	// Queue event signaler
	boost::condition_variable space_available_;
	boost::condition_variable data_available_;
	boost::condition_variable finished_;
};

} /* namespace libav_image_transport */

#endif /* LIBAV_IMAGE_TRANSPORT__WORKER_HPP_ */
