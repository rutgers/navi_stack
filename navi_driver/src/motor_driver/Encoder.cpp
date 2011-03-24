/*
 * Encoder.cpp
 *
 *  Created on: Dec 2, 2009
 *      Author: asher
 */

#include "Encoder.h"

Encoder::Encoder() {
	// TODO Auto-generated constructor stub
	clearCount();
}

void Encoder::clearCount(){
	encoderCount=0;
}


void Encoder::update(char channelA, char channelB){
	   if (!(priorA ) && (channelA)) {
	     if (!channelB) {
	       encoderCount--;
	     } else {
	       encoderCount++;
	     }
	   }
	   if ((priorA) && !(channelA)) {
	     if (!channelB ) {
	       encoderCount++;
	     } else {
	       encoderCount--;
	     }
	   }

	   priorA  = channelA;

	  if (!(priorB) && (channelB)) {
	   	     if (!channelA) {
	   	       encoderCount++;
	   	     } else {
	   	       encoderCount--;
	   	     }
	   	   }
	   	   if ((priorB) && (!channelB)) {
	   	     if (!channelA ) {
	   	       encoderCount--;
	   	     } else {
	   	       encoderCount++;
	   	     }
	   	   }
	   priorB = channelB;

}

int Encoder::count(){
	return encoderCount;
}

Encoder::~Encoder() {
	// TODO Auto-generated destructor stub
}
