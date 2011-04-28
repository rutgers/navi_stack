/*
 * Encoder.h
 *
 *  Created on: Apr 27, 2011
 *      Author: asher
 */

#ifndef ENCODER2_H_
#define ENCODER2_H_

class Encoder {
public:
	Encoder();
	int count();
	void adjustCount(int amount); //if you need to adjust the count for sum reason
								//(putting this in for blacklash adjustments
	void clearCount();
	void update(char channelA, char channelB);
	 ~Encoder();
private:
	 int priorA; //prior A value
	 int priorB;
	 int encoderCount;

};


#endif /* ENCODER_H_ */
