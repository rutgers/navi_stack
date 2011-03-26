/*
 * Encoder.h
 *
 *  Created on: Dec 2, 2009
 *      Author: asher
 */

#ifndef ENCODER_COUNTER_H_
#define ENCODER_COUNTER_H_

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
	 char priorA; //prior A value
	 char priorB;
	 int encoderCount;

};






#endif /* ENCODER_H_ */
