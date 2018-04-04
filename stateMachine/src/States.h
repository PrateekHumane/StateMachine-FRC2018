/*
 * States.h
 *
 *  Created on: Feb 14, 2018
 *      Author: team 41 robotics
 */

#ifndef SRC_STATES_H_
#define SRC_STATES_H_

#define USE_CONTROLLER 0

//#define BACKUP_CONTROLLER 1

#ifndef BACKUP_CONTROLLER
//easily call the buttons. we need to map these values.
enum BUTTON_BOARD {
	//AXES
	SLIDER_1 = 4, //Robot throttle
	SLIDER_2 = 3, //Lift throttle
	SLIDER_3 = 1, //Moves lift
	SLIDER_4 = 0,
	KNOB_LEFT = 5,
	KNOB_RIGHT = 2,
	//BUTTONS
	BUTTON_LEFT_UP = 2, //Push cube out
	BUTTON_LEFT_LEFT = 1, //Rotate cube left
	BUTTON_LEFT_DOWN = 3, //Pull cube in
	BUTTON_LEFT_RIGHT = 4, //Rotate cube right
	BUTTON_RIGHT_UP = 8,
	BUTTON_RIGHT_LEFT = 7,
	BUTTON_RIGHT_DOWN = 9,
	BUTTON_RIGHT_RIGHT = 10,
	ROCKER_1_UP = 24, //Move claw out (x-axis)
	ROCKER_1_DOWN = 23, //Moves claw in (x-axis)
	ROCKER_2_UP = 22,
	ROCKER_2_DOWN = 21,
	ROCKER_3_UP = 20,
	ROCKER_3_DOWN = 19,
	ROCKER_4_UP = 18, //Moves lift up
	ROCKER_4_DOWN = 17, //Moves lift down
	ROCKER_5 = 28, //Moves lift to bottom
	ROCKER_6 = 27, //Moves lift to exchange height
	ROCKER_7 = 26, //Moves lift to switch height
	ROCKER_8 = 25, //Moves lift to scale height
	TOGGLE_LEFT = 5,
	TOGGLE_RIGHT = 6

};

#else
//Only for emergencies!
enum BUTTON_BOARD {
	//AXES - Not sure how to do sliders with the controller
	SLIDER_1 = 0, //Robot throttle
	SLIDER_2 = 0, //Lift throttle
	SLIDER_3 = 0, //Moves lift
	SLIDER_4 = 0,
	KNOB_LEFT = 0,
	KNOB_RIGHT = 0,
	//BUTTONS
	BUTTON_LEFT_UP = 4, //Push cube out				Y button
	BUTTON_LEFT_LEFT = 3, //Rotate cube left		X button
	BUTTON_LEFT_DOWN = 1, //Pull cube in			A button
	BUTTON_LEFT_RIGHT = 2, //Rotate cube right		B button
	BUTTON_RIGHT_UP = 0,
	BUTTON_RIGHT_LEFT = 0,
	BUTTON_RIGHT_DOWN = 0,
	BUTTON_RIGHT_RIGHT = 0,
	ROCKER_1_UP = 10, //Move claw out (x-axis)		Right joystick button
	ROCKER_1_DOWN = 9, //Moves claw in (x-axis)		Left joystick button
	ROCKER_2_UP = 0,
	ROCKER_2_DOWN = 0,
	ROCKER_3_UP = 0,
	ROCKER_3_DOWN = 0,
	ROCKER_4_UP = 6, //Moves lift up				Right bumper
	ROCKER_4_DOWN = 5, //Moves lift down			Left bumper
	ROCKER_5 = 7, //Moves lift to bottom			Back button
	ROCKER_6 = 0, //Moves lift to exchange height
	ROCKER_7 = 0, //Moves lift to switch height
	ROCKER_8 = 8, //Moves lift to scale height		Start button
	TOGGLE_LEFT = 0,
	TOGGLE_RIGHT = 0
};
#endif

//TODO: make enums for all of the inputs/outputs.



#endif /* SRC_STATES_H_ */
