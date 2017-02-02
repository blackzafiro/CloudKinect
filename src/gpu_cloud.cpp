/** @file
 * Simple demo of how to read point cloud from Kinect and access it
 * with cuda.
 */

#include <iostream>
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>

/**
 * Shows how to get the color and cloud points from the kinect.
 */
int main(int argc, char *argv[])
{
//- [context]
    libfreenect2::Freenect2 freenect2;
    libfreenect2::Freenect2Device *dev = 0;
    libfreenect2::PacketPipeline *pipeline = 0;
//- [context]

    std::string serial = "";
    int gpuDeviceId = 0; // TODO: gpus válidos?
    
#ifdef LIBFREENECT2_WITH_CUDA_SUPPORT
    std::cout << "\x1b[0;36m" << "Creating CUDA pipeline..." << "\x1b[0m" << std::endl;
    if(!pipeline)
        pipeline = new libfreenect2::CudaPacketPipeline(gpuDeviceId);
    std::cout << "Pipeline " << pipeline << std::endl;
#else
    std::cerr << "CUDA pipeline is not supported!" << std::endl;
#endif
    
//- [discovery]
    std::cout << "Searching for device..." << std::endl;
    if(freenect2.enumerateDevices() == 0)
    {
        std::cerr << "no device connected!" << std::endl;
        return -1;
    }

    std::cout << "Getting device number..." << std::endl;
    if (serial == "")
    {
        serial = freenect2.getDefaultDeviceSerialNumber();
    }
//- [discovery]

    if(pipeline)
    {
//- [open]
        std::cout << "Opening with pipeline..." << std::endl;
        dev = freenect2.openDevice(serial, pipeline);
//- [open]
    }
    else
    {
        std::cout << "Opening with serial..." << std::endl;
        dev = freenect2.openDevice(serial);
    }

    if(dev == 0)
    {
        std::cerr << "failure opening device!" << std::endl;
        return -1;
    }
    
//- [start]
    if (!dev->start())
    {
        std::cerr << "Could not start device!" << std::endl;
    }
    std::cout << "device serial: " << dev->getSerialNumber() << std::endl;
    std::cout << "device firmware: " << dev->getFirmwareVersion() << std::endl;
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
    
//- [registration]
    registration->apply(rgb, depth, &undistorted, &registered);
//- [registration]

    std::cout << "Captured one frame..." << std::endl;

    listener.release(frames);
    
//- [capture one frame]
        
    std::cout << "Stopping and closing..." << std::endl;
    dev->stop();
    dev->close();
    
    return 0;
}
