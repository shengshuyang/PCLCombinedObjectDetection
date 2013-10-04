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

#include <iostream>
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <csignal>
#include <unistd.h>
#include "pcdBuffer.h"

pcdBuffer buff;
bool isDone = false;
boost::mutex ioMutex;

void grabberCallBack(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& cloud)
{
	if (!buff.pushBack(cloud))
	{
		{
			boost::mutex::scoped_lock ioLock(ioMutex);
			std::cout << "Warning! Buffer was full, overwriting data" << std::endl;
		}
	}
}

// Procuder thread function
void grabAndSend()
{
	pcl::Grabber* interface = new pcl::OpenNIGrabber();
	boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& )> f = 
		boost::bind(&grabberCallBack, _1);
	interface->registerCallback(f);
	interface->start();
	while(true)
	{
		if (isDone)
			break;
		boost::this_thread::sleep(boost::posix_time::seconds(1));
	}
	interface->stop();
}

void writeToDisk(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& cloud)
{
	static int counter = 1;
	pcl::PCDWriter w;
	std::stringstream ss;
	ss << counter;
	std::string prefix = "input-";
	std::string ext = ".pcd";
	std::string fname = prefix + ss.str() + ext; 
	w.writeBinaryCompressed(fname, *cloud);
	counter++;
}

// Consumer thread function
void receiveAndProcess()
{
	while(true)
	{
		if (isDone)
			break;
		writeToDisk(buff.getFront());
	}

	{
		boost::mutex::scoped_lock ioLock(ioMutex);
		std::cout << "Writing remaing " << buff.getSize() << " clouds in the buffer to disk..." << std::endl;
	}
	while(!buff.isEmpty())
	{
		writeToDisk(buff.getFront());
	}
}

void ctrl_C(int dummy)
{
	boost::mutex::scoped_lock ioLock(ioMutex);
	std::cout << std::endl << "Ctrl-C detected, exit condition set to true" << std::endl;
	isDone = true;
}

int main(int argc, char** argv)
{
	if (argc < 2)
	{
		std::cerr << "Usage: " << argv[0] << " bufferSize " << std::endl;
		exit(1);
	}
	int buffSize = atoi(argv[1]);
	buff.setCapacity(buffSize);
	std::cout << "Starting the producer and consumer threads..." << std::endl;
	std::cout << "Press Ctrl-C to end" << std::endl;
	boost::thread producer(grabAndSend);
	boost::this_thread::sleep(boost::posix_time::seconds(2));
	boost::thread consumer(receiveAndProcess);
	signal(SIGINT, ctrl_C);
	producer.join();
	{
		boost::mutex::scoped_lock ioLock(ioMutex);
		std::cout << "Producer done" << std::endl;
	}
	consumer.join();
	std::cout << "Consumer done" << std::endl;
	return 0;
}
