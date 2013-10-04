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

#include "pcdBuffer.h"
#include <iostream>

pcdBuffer::pcdBuffer()
{
}

bool pcdBuffer::pushBack(pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud)
{
	bool retVal = false;
	{
		boost::mutex::scoped_lock buffLock(bmutex);
		if (!buffer.full())
		{
			retVal = true;
		}
		buffer.push_back(cloud);
	}
	buffEmpty.notify_one();
	return retVal;
}

pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr pcdBuffer::getFront()
{
	pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud;
	{
		boost::mutex::scoped_lock buffLock(bmutex);
		while(buffer.empty())
		{
			if (isDone)
				break;
			{
				boost::mutex::scoped_lock ioLock(ioMutex);
				std::cout << "Buffer is empty" << std::endl;
			}
			buffEmpty.wait(buffLock);
		}
		cloud = buffer.front();
		buffer.pop_front();
	}
	return cloud;
}

bool pcdBuffer::isFull()
{
	boost::mutex::scoped_lock buffLock;
	return buffer.full();
}

bool pcdBuffer::isEmpty()
{
	boost::mutex::scoped_lock buffLock;
	return buffer.empty();
}

void pcdBuffer::setCapacity(int buffSize)
{
	boost::mutex::scoped_lock buffLock;
	buffer.set_capacity(buffSize);
}

int pcdBuffer::getSize()
{
	boost::mutex::scoped_lock buffLock;
	return buffer.size();
}

int pcdBuffer::getCapacity()
{
	return buffer.capacity();
}

