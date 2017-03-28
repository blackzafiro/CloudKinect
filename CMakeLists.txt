CMAKE_MINIMUM_REQUIRED(VERSION 2.8.12.1)
include(AuxFuncs.cmake)

### Project
PROJECT(CloudKinect)

set(INC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)

### Dependencies

set(freenect2_DIR "/home/blackzafiro/Programas/libfreenect2/build/lib/cmake/freenect2")
set(PCL_DIR "/home/blackzafiro/Descargas/Desarrollo/pcl-trunk/build/share/pcl-1.8")
set(OpenCV_DIR "/home/blackzafiro/Descargas/Desarrollo/OpenCV/opencv-3.2.0/build/share/OpenCV")
list(APPEND CMAKE_MODULE_PATH "${PCL_DIR}")

find_package( PkgConfig )
pkg_check_modules( EIGEN3 REQUIRED eigen3 )

find_package(CUDA REQUIRED)
find_package(freenect2 REQUIRED)
find_package(PCL 1.8 REQUIRED COMPONENTS common io visualization cuda_common cuda_io)
find_package(OpenCV REQUIRED core imgproc highgui cudaarithm)

### APP

file(GLOB srcs ./src/*.cpp ${INC_DIR}/*.h)

# Print in color
add_library(TermColorPrint STATIC ./src/TermColorPrint/TermColorPrint.cpp)

# Executable called "gpu_cloud" built from sourcefiles
add_executable(gpu_cloud ${srcs})

# Indicate where include files are
include_directories(${INC_DIR}
                    ${INC_DIR}/TermColorPrint
                    ${freenect2_INCLUDE_DIR}
                    ${PCL_INCLUDE_DIRS}
                    ${PCL_CUDA_INCLUDE_DIRS}
                    ${CUDA_INCLUDE_DIRS}
                    ${EIGEN3_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Link executable to libraries
target_link_libraries(gpu_cloud
                      TermColorPrint
                      ${freenect2_LIBRARIES}
                      ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_VISUALIZATION_LIBRARIES} ${PCL_CUDA_LIBRARIES} ${PCL_CUDA_IO_LIBRARIES}
                      ${OpenCV_LIBS}
                      )
					  
					  
### DOCS

ADD_SUBDIRECTORY(${CMAKE_SOURCE_DIR}/doc)