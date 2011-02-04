#cmake_minimum_required(VERSION 2.4.6)
cmake_minimum_required(VERSION 2.8)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})


set(CMAKE_CONFIGURATION_TYPES "Debug;Release" CACHE STRING "Configs" FORCE)

find_package(CUDA REQUIRED)

set(CUDA_COMPUTE_CAPABILITIES " 1.0 1.1 1.2 1.3 2.0 " CACHE STRING "Add or remove compute capability")
set(CUDA_NVCC_FLAGS_ARCH ${CUDA_COMPUTE_CAPABILITIES})

set(CUDA_NVCC_FLAGS_NUM "")

while(NOT ${CUDA_NVCC_FLAGS_ARCH} STREQUAL "")
	string(REGEX MATCH "[0-9]+.[0-9]+" RESULT_NUM ${CUDA_NVCC_FLAGS_ARCH})
	string(REGEX MATCHALL "[0-9]" RESULT_STR ${RESULT_NUM})
	string(REGEX REPLACE ";" "\ " RESULT ${RESULT_STR})
	list(APPEND CUDA_NVCC_FLAGS_NUM ${RESULT})
	string(REGEX REPLACE "${RESULT_NUM}" "\ " CUDA_NVCC_FLAGS_ARCH_STR ${CUDA_NVCC_FLAGS_ARCH})
	string(STRIP ${CUDA_NVCC_FLAGS_ARCH_STR} CUDA_NVCC_FLAGS_ARCH)
endwhile()
	
if (UNIX OR APPLE)
	set (CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS}  "-Xcompiler;-fPIC;")
	#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}" "-fPIC")
endif()

file(GLOB int_hdrs "src/*.h*")
file(GLOB srcs "src/*.cpp")

file(GLOB cuda "src/*.cu")
file(GLOB hdrs "include/stereo_gpu/*.h*")

source_group("Host" FILES ${srcs} ${hdrs} ${int_hdrs})
source_group("Cuda" FILES ${cuda})

include_directories(${CUDA_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/include/stereo_gpu)

CUDA_COMPILE(cuda_objs ${cuda})
rosbuild_add_library(${PROJECT_NAME} ${srcs} ${hdrs} ${cuda} ${cuda_objs} ${int_hdrs})
target_link_libraries(${PROJECT_NAME} ${CUDA_LIBRARIES}) 
#set_target_properties(${the_target} PROPERTIES DEBUG_POSTFIX "-d" DEFINE_SYMBOL "API_EXPORTS")


rosbuild_add_executable(test_stereo_gpu test/main.cpp)
add_dependencies(test_stereo_gpu ${PROJECT_NAME})
target_link_libraries(test_stereo_gpu ${PROJECT_NAME}) 

