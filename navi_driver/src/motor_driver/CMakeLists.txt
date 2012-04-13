cmake_minimum_required(VERSION 2.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()

#set your project name
Project(navi_motor_driver)
set(ARDUINO_CORE_DIR  arduino)

#Include the arduino board specific cmake file
include(${PROJECT_SOURCE_DIR}/cmake_scripts/arduino_diecimila.cmake)
include_directories(${PROJECT_SOURCE_DIR} muc/)


file(GLOB AVR_ROS_SRC
    "avr_ros/*.cpp"
)


#Add of your own avr c++ files to this list
set(PROJECT_SRC
    main.cpp
    avr_ros_user.cpp
	motorcontrol.cpp
	Encoder.cpp
	robot.cpp
    ${AVR_ROS_SRC}
    ${ARDUINO_SOURCE_FILES}
)

rosbuild_find_ros_package(avr_bridge)

add_custom_command(
	OUTPUT  ${avr_bridge_PACKAGE_PATH}/avr_ros/*.{c,h,cpp,cc,hpp}
	COMMAND ${avr_bridge_PACKAGE_PATH}/scripts/gen_avr.py
	${CMAKE_SOURCE_DIR}
	DEPENDS ${avr_bridge_PACKAGE_PATH}/**
	)

add_executable(firmware ${PROJECT_SRC}
	${avr_bridge_PACKAGE_PATH}/avr_ros/*.{c,h,cpp,cc,hpp})

#to compile, use make
#to program arduino on /dev/ttyUSB0, do make flash
#to check the program size use make size
