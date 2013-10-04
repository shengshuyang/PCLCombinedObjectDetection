/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2012 Sudarshan Srinivasan <sudarshan85@gmail.com>
 * 
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 * 
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef PCDBUFFER_H_
#define PCDBUFFER_H_

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>
#include <boost/circular_buffer.hpp>

class pcdBuffer
{
	public:
		pcdBuffer();
		bool pushBack(pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr); // thread-save wrapper for push_back() method of ciruclar_buffer
		pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr getFront(); // thread-save wrapper for front() method of ciruclar_buffer
		bool isFull();
		bool isEmpty();
		int getSize();
		int getCapacity();
		void setCapacity(int buffSize); 
	private:
		pcdBuffer(const pcdBuffer&); // Disabled copy constructor
		pcdBuffer& operator = (const pcdBuffer&); // Disabled assignment operator
		boost::mutex bmutex;
		boost::condition_variable buffEmpty;
		boost::circular_buffer<pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr> buffer;
};

extern bool isDone; // variable set to true on Ctrl-C detection
extern boost::mutex ioMutex;

#endif
