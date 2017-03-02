 /*
  * Software License Agreement (BSD License)
  *
  *  Copyright (c) 2017 UNAM, Mex.
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
  *   * Neither the name of UNAM nor the names of its
  *     contributors may be used to endorse or promote products derived
  *     from this software without specific prior written permission.
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

/** @file
 * Simple demo of how to read point cloud from Kinect and access it
 * with cuda.
 */

#include <iostream>
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>

//#include <pcl/cuda/point_cloud.h>
//#include <pcl/cuda/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
//#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>

#include "TermColorPrint/TermColorPrint.h"
//#include <string>

/** Returns the corresponding string for the format enumeration. */
std::string format_name(libfreenect2::Frame::Format f)
{
	switch (f) {
	case libfreenect2::Frame::Invalid:
		return "Invalid";
	case libfreenect2::Frame::Raw:
		return "Raw";
	case libfreenect2::Frame::Float:
		return "Float";
	case libfreenect2::Frame::BGRX:
		return "BGRX";
	case libfreenect2::Frame::RGBX:
		return "RGBX";
	case libfreenect2::Frame::Gray:
		return "Gray";
	}
	return "This shouldn't happen";
}

/** Prints info about the frame and data. */
void print_frame(libfreenect2::Frame& frame, bool print_data)
{
	std::cout << "[" << frame.width << ", " << frame.height << "]" << " format: " << format_name(frame.format) << std::endl;
    if (print_data)
    {
		for(int i=0; i<frame.height; i++)
		{
			for(int j=0; j<frame.width; j++)
			{
				std::cout << i << ","  << j << " -> (" << int(frame.data[i * frame.bytes_per_pixel + j]) << ")\t";
			}
			std::cout << std::endl;
		}
    }
}

void show_depth_cloud(libfreenect2::Frame *frame)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cpu_cloud(new pcl::PointCloud<pcl::PointXYZ>);

	cpu_cloud->width = frame->width;
	cpu_cloud->height = frame->height;
	cpu_cloud->is_dense = true;
	cpu_cloud->points.resize(cpu_cloud->width * cpu_cloud->height);
	int step = frame->width;
	int index;
	for(int i=0; i < frame->height; i++)
	{
		for(int j=0; j < frame->width; j++)
		{
			index = i * step + j;
			cpu_cloud->points[index].x = j;
			cpu_cloud->points[index].y = i;
			cpu_cloud->points[index].z = (int)(frame->data[i * frame->width + j]);
		}
	}
	pcl::visualization::CloudViewer viewer("Kinect V2: Depth");
	viewer.showCloud(cpu_cloud);
	while(!viewer.wasStopped()) {}
}

void show_registered_cloud(libfreenect2::Frame *frame)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cpu_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	cpu_cloud->width = frame->width;
	cpu_cloud->height = frame->height;
	cpu_cloud->is_dense = true;
	cpu_cloud->points.resize(cpu_cloud->width * cpu_cloud->height);
	int step = frame->width;
	int index;
	uint8_t r = 20, g = 10, b = 255;
	uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
	float color = *reinterpret_cast<float*>(&rgb);
	for(int i=0; i < frame->height; i++)
	{
		for(int j=0; j < frame->width; j++)
		{
			index = i * step + j;
			cpu_cloud->points[index].x = j;
			cpu_cloud->points[index].y = i;
			cpu_cloud->points[index].z = (int)(frame->data[i * frame->width + j]);
			cpu_cloud->points[index].rgb = color;
		}
	}
	pcl::visualization::CloudViewer viewer("Kinect V2");
	viewer.showCloud(cpu_cloud);
	while(!viewer.wasStopped()) {}
}


/**
 * Shows how to get the color and cloud points from the kinect.
 */
int main(int argc, char *argv[])
{
	PrettyPrint::ColorPrinter cout(std::cout, PrettyPrint::Cyan);
	PrettyPrint::ColorPrinter cerr(std::cout, PrettyPrint::Red);
    
//- [context]
    libfreenect2::Freenect2 freenect2;
    libfreenect2::Freenect2Device *dev = 0;
    libfreenect2::PacketPipeline *pipeline = 0;
//- [context]

    std::string serial = "";
    int gpuDeviceId = 0; // TODO: gpus válidos?
    
#ifdef LIBFREENECT2_WITH_CUDA_SUPPORT
    cout << "Creating CUDA pipeline..." << std::endl;
    if(!pipeline)
        pipeline = new libfreenect2::CudaPacketPipeline(gpuDeviceId);
    cout << "Pipeline " << pipeline << std::endl;
#else
    cerr << "CUDA pipeline is not supported!" << std::endl;
#endif
    
//- [discovery]
    cout << "Searching for device..." << std::endl;
    if(freenect2.enumerateDevices() == 0)
    {
        cerr << "no device connected!" << std::endl;
        return -1;
    }

    cout << "Getting device number..." << std::endl;
    if (serial == "")
    {
        serial = freenect2.getDefaultDeviceSerialNumber();
    }
//- [discovery]

    if(pipeline)
    {
//- [open]
        cout << "Opening with pipeline..." << std::endl;
        dev = freenect2.openDevice(serial, pipeline);
//- [open]
    }
    else
    {
        cout << "Opening with serial..." << std::endl;
        dev = freenect2.openDevice(serial);
    }

    if(dev == 0)
    {
        cerr << "failure opening device!" << std::endl;
        return -1;
    }
    
//- [start]
    if (!dev->start())
    {
        cerr << "Could not start device!" << std::endl;
    }
    cout << "device serial: " << dev->getSerialNumber() << std::endl;
    cout << "device firmware: " << dev->getFirmwareVersion() << std::endl;
//- [start]
    
//- [registration setup]
    // Optional
    // Combines frames of depth and color camera with default reversed ingeniered parameters.
    // Can be replaced.
    libfreenect2::Registration* registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
    libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4);
//- [registration setup]

//- [listeners]
    int types = 0;
    // rgb
    types |= libfreenect2::Frame::Color;
    // depth
    types |= libfreenect2::Frame::Ir | libfreenect2::Frame::Depth;
    
    libfreenect2::SyncMultiFrameListener listener(types);
    libfreenect2::FrameMap frames;

    dev->setColorFrameListener(&listener);
    dev->setIrAndDepthFrameListener(&listener);
//- [listeners]

//- [capture one frame]
    if (!listener.waitForNewFrame(frames, 10*1000)) // 10 sconds
    {
        std::cout << "timeout!" << std::endl;
        return -1;
    }
    libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
    libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
    libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];
    
    cout << "Rgb " << std::endl;
    print_frame(*rgb, false);
    cout << "Ir " << std::endl;
    print_frame(*ir, false);
    cout << "Depth " << std::endl;
    print_frame(*depth, false);
    
    show_depth_cloud(depth);


//- [registration]
    registration->apply(rgb, depth, &undistorted, &registered);
//- [registration]

    cout << "Captured one frame..." << std::endl;
    cout << "Undistorted: " << undistorted.status << " format: " << format_name(undistorted.format) << std::endl;
    cout << "Registered: " << registered.status << " format: " << format_name(registered.format) << std::endl;

    show_registered_cloud(&registered);

    listener.release(frames);
    
//- [capture one frame]
        
    cout << "Stopping and closing..." << std::endl;
    dev->stop();
    dev->close();
    
    delete registration;

    return 0;
}
