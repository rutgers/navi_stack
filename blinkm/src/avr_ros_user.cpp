/*
 * avrRos_user.cpp
 *
 *  Created on: Jan 4, 2011
 *      Author: asher
 */
#include "WProgram.h"
#include <stdio.h>
#include "avr_ros/ros.h"

namespace ros {
    int fputc(char c, FILE *stream) {
        Serial.write(c);
        return 0;
    }
    int fgetc(FILE * stream){
		return Serial.read();
	}
}
