#include <iostream>
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/packet_pipeline.h>

int main(int argc, char *argv[])
{
/// [context]
    libfreenect2::Freenect2 freenect2;
    libfreenect2::Freenect2Device *dev = 0;
    libfreenect2::PacketPipeline *pipeline = 0;
/// [context]

    std::string serial = "";
    int gpuDeviceId = 0; // TODO: gpus válidos?
    
#ifdef LIBFREENECT2_WITH_CUDA_SUPPORT
    std::cout << "\x1b[0;36m" << "Creating CUDA pipeline..." << "\x1b[0m" << std::endl;
    if(!pipeline)
        pipeline = new libfreenect2::CudaPacketPipeline(gpuDeviceId);
    std::cout << "Pipeline " << pipeline << std::endl;
#else
    std::cout << "CUDA pipeline is not supported!" << std::endl;
#endif
    
/// [discovery]
    std::cout << "Searching for device..." << std::endl;
    if(freenect2.enumerateDevices() == 0)
    {
        std::cout << "no device connected!" << std::endl;
        return -1;
    }

    std::cout << "Getting device number..." << std::endl;
    if (serial == "")
    {
        serial = freenect2.getDefaultDeviceSerialNumber();
    }
/// [discovery]

    if(pipeline)
    {
/// [open]
        std::cout << "Opening with pipeline..." << std::endl;
        dev = freenect2.openDevice(serial, pipeline);
/// [open]
    }
    else
    {
        std::cout << "Opening with serial..." << std::endl;
        dev = freenect2.openDevice(serial);
    }

    if(dev == 0)
    {
        std::cout << "failure opening device!" << std::endl;
        return -1;
    }
    
    std::cout << "Stopping and closing..." << std::endl;
    dev->stop();
    dev->close();
    
    return 0;
}
