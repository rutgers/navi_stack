#include "WProgram.h" //include the Arduino library
#include <stdio.h>

#include <avr_ros/ros.h>
#include <avr_ros/StatusLight.h>

#include <avr/interrupt.h>
#include <avr/delay.h>

#include "BlinkM_funcs.h"

blinkm::StatusLight status_light;

const byte blinkm_addr = 0x09;


void toggle()
{ //toggle an led to debug the program
    static char t=0;
    if (!t ) {
        digitalWrite(13, HIGH);   // set the LED on
        t = 1;
    } else {
        digitalWrite(13, LOW);    // set the LED off
        t = 0;
    }
}


void blink( bool solid,  char color)
{
       	if (solid)
	{
           switch (color)
		{

		case 'r': //RED
			BlinkM_stopScript( blinkm_addr );
			BlinkM_fadeToRGB( blinkm_addr, 0xff,0,0);  
			break;

		case 'g':// Green
			BlinkM_stopScript( blinkm_addr );
			BlinkM_fadeToRGB( blinkm_addr, 0,0xff,0);
			break;

		case 'b':// Blue
			BlinkM_stopScript( blinkm_addr );
			BlinkM_fadeToRGB( blinkm_addr, 0,0,0xff);
			break;

		case 'c':// Cyan
			BlinkM_stopScript( blinkm_addr );
			BlinkM_fadeToRGB( blinkm_addr, 0,0xff,0xff);
			break;

		case 'm': // Magenta
			BlinkM_stopScript( blinkm_addr );
			BlinkM_fadeToRGB( blinkm_addr, 0xff,0,0xff);
			break;

		case 'y': // yellow
                        BlinkM_stopScript( blinkm_addr );
			BlinkM_fadeToRGB( blinkm_addr, 0xff,0xff,0);
			break;

		default: // Black
			BlinkM_stopScript( blinkm_addr );
			BlinkM_fadeToRGB( blinkm_addr, 0,0,0);
			break;
		}
	}


	else
	{
               	switch (color)
		{
		case 'r':  // Blink Red
                        BlinkM_stopScript( blinkm_addr );
			BlinkM_playScript( blinkm_addr, 3,0,0 );
			break;

		case 'g':  // Blink Green
			BlinkM_stopScript( blinkm_addr );
			BlinkM_playScript( blinkm_addr, 4,0,0 );
			break;

		case 'b': // Blink Blue
			BlinkM_stopScript( blinkm_addr );
			BlinkM_playScript( blinkm_addr, 5,0,0 );
			break;

		case 'c': //Blink Cyan
			BlinkM_stopScript( blinkm_addr );
			BlinkM_playScript( blinkm_addr, 6,0,0 );
			break;

		case 'm': //Blink Magenta
			BlinkM_stopScript( blinkm_addr );
			BlinkM_playScript( blinkm_addr, 7,0,0 );
			break;

		case 'y': //Blink Yellow
                        BlinkM_stopScript( blinkm_addr );
			BlinkM_playScript( blinkm_addr, 8,0,0 );
			break;

		default: //OFF
			BlinkM_stopScript( blinkm_addr );
			BlinkM_playScript( blinkm_addr, 9,0,0 );
			break;
		}

	}

}

void light_cb(const ros::Msg* msg){
	toggle();
	blink(status_light.solid, status_light.color);
}


// Since we are hooking into a standard
// arduino sketch, we must define our program in
// terms of the arduino setup and loop functions.

void setup()
{
    Serial.begin(57600);
    pinMode(13, OUTPUT); //set up the LED

	blink(false, 0);
    node.subscribe("status_light",light_cb, &status_light);

}



void loop()
{
	 node.spin();
	 
	_delay_ms(1);
}
