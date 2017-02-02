CMAKE_MINIMUM_REQUIRED(VERSION 2.8.12.1)

### Project
PROJECT(CloudKinect)

set(INC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)

### Dependencies

set(FREENECT2_cmake "/home/blackzafiro/Descargas/Desarrollo/libfreenect2/build/freenect2/lib/cmake/freenect2")
LIST(APPEND CMAKE_MODULE_PATH "${FREENECT2_cmake}")

MESSAGE(STATUS ${CMAKE_MODULE_PATH})

find_package(freenect2 REQUIRED)

### APP

file(GLOB srcs ./src/*.cpp ${INC_DIR}/*.h)

# Executable called "gpu_cloud" built from sourcefiles
add_executable(gpu_cloud ${srcs})

# Indicate where include files are
include_directories(${INC_DIR}
                    ${freenect2_INCLUDE_DIR})

# Link executable to libraries
target_link_libraries(gpu_cloud
					  ${freenect2_LIBRARIES})
					  
					  
### DOCS

ADD_SUBDIRECTORY(${CMAKE_SOURCE_DIR}/doc)