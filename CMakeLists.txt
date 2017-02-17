CMAKE_MINIMUM_REQUIRED(VERSION 2.8.12.1)

### Color aux
if(NOT WIN32)
  string(ASCII 27 Esc)
  set(ColourReset "${Esc}[m")
  set(ColourBold  "${Esc}[1m")
  set(Red         "${Esc}[31m")
  set(Green       "${Esc}[32m")
  set(Yellow      "${Esc}[33m")
  set(Blue        "${Esc}[34m")
  set(Magenta     "${Esc}[35m")
  set(Cyan        "${Esc}[36m")
  set(White       "${Esc}[37m")
  set(BoldRed     "${Esc}[1;31m")
  set(BoldGreen   "${Esc}[1;32m")
  set(BoldYellow  "${Esc}[1;33m")
  set(BoldBlue    "${Esc}[1;34m")
  set(BoldMagenta "${Esc}[1;35m")
  set(BoldCyan    "${Esc}[1;36m")
  set(BoldWhite   "${Esc}[1;37m")
endif()
function(colormessage)
  list(GET ARGV 0 MessageType)
  if(MessageType STREQUAL FATAL_ERROR OR MessageType STREQUAL SEND_ERROR)
    list(REMOVE_AT ARGV 0)
    message(${MessageType} "${BoldRed}${ARGV}${ColourReset}")
  elseif(MessageType STREQUAL WARNING)
    list(REMOVE_AT ARGV 0)
    message(${MessageType} "${BoldYellow}${ARGV}${ColourReset}")
  elseif(MessageType STREQUAL AUTHOR_WARNING)
    list(REMOVE_AT ARGV 0)
    message(${MessageType} "${BoldCyan}${ARGV}${ColourReset}")
  elseif(MessageType STREQUAL STATUS)
    list(REMOVE_AT ARGV 0)
    message(${MessageType} "${Green}${ARGV}${ColourReset}")
  else()
    message("${ARGV}")
  endif()
endfunction()


### Project
PROJECT(CloudKinect)

set(INC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)

### Dependencies

set(freenect2_DIR "/home/blackzafiro/Descargas/Desarrollo/libfreenect2/build/freenect2/lib/cmake/freenect2/")
set(PCL_DIR "/home/blackzafiro/Descargas/Desarrollo/pcl-trunk/build/share/pcl-1.8/PCLConfig.cmake")
set(PCL_HOME "/home/blackzafiro/Descargas/Desarrollo/pcl-trunk/build")
set(PCL_CUDA_INCLUDE_DIRS ${PCL_HOME}/include/pcl-1.8)

find_package(CUDA REQUIRED)
find_package(freenect2 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common cuda_common cuda_io)

message(STATUS ${PCL_INCLUDE_DIRS})
MESSAGE(STATUS ${PCL_LIBRARIES})
message(STATUS ${PCL_CUDA_INCLUDE_DIRS})

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
                    ${CUDA_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Link executable to libraries
target_link_libraries(gpu_cloud
                      TermColorPrint
                      ${freenect2_LIBRARIES}
                      ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_CUDA_LIBRARIES} ${PCL_CUDA_IO_LIBRARIES}
                      )
					  
					  
### DOCS

ADD_SUBDIRECTORY(${CMAKE_SOURCE_DIR}/doc)