/*
 * stateMachine.h
 *
 *  Created on: Feb 16, 2018
 *      Author: RoboWarriors
 */

#ifndef SRC_STATEMACHINE_H_
#define SRC_STATEMACHINE_H_

#include <map>
#include "Driving.h"
#include "Lift.h"

#define evtRef eventData::
#define evtVal eventData::eventTypeValues
#define evtType eventData::eventTypes
#define IS_STATE(x) x >= 0 && x <= 3

class eventData{
public:
	enum eventTypes{
		switchSide, scaleSide, switchDone, scaleDone, currentState
	};
	enum eventTypeValues{
		DONT_CARE, IS, IS_NOT
	};
	std::array<uint8_t, 5> events;

	eventData(){
		events[switchSide] = DONT_CARE;
		events[scaleSide] = DONT_CARE;
		events[switchDone] = IS_NOT;
		events[scaleDone] = IS_NOT;
	}
	~eventData(){};
};

class stateMachine
{
public:
	stateMachine(const char* gameSpecificMessage, char startSide, Driving *drvPointer, Lift *lftPointer);
	void runState();
	Timer * timer;
private:
	Driving * drv;
	Lift *lft;
	eventData *eData;

	enum state{
		START,
		SWITCH,
		SCALE,
		PICKUP,
		MIDDLE
	};

	typedef enum transitions{
		DRIVE_FORWARD_SCALE,
		DRIVE_ACROSS_SWITCH,
		DRIVE_FORWARD_SWITCH,
		SWITCH_TO_CUBE,
		SCALE_TO_CUBE,
		CUBE_ACROSS_SWITCH,
		CUBE_TO_SWITCH,
		CUBE_ACROSS_SCALE,
		CUBE_TO_SCALE
	};

	struct eventDataComp
	{
	   bool operator() (const std::array<uint8_t, 5> &lhs, const std::array<uint8_t, 5> &rhs) const
	   {
	       for (int i = 0; i < lhs.size(); i++){
	    	   if (! (lhs[i] == evtVal::DONT_CARE || rhs[i] == evtVal::DONT_CARE || lhs[i] == rhs[i]) ) return true;
	       }
	       return false;
	   }
	};

	std::map<std::array<uint8_t, 5>, transitions> event_to_transitions;

	std::array<std::array<uint8_t, 5>, 10> eventKeys = {{
//							  	switch side			scale side			switch done			scale done			current state
/*DRIVE_FORWARD_SCALE*/		{{	evtVal::DONT_CARE,	evtVal::IS,			evtVal::DONT_CARE,	evtVal::DONT_CARE,	state::START}},
/*DRIVE_ACROSS_SWITCH*/		{{	evtVal::IS_NOT,		evtVal::IS_NOT,		evtVal::DONT_CARE,	evtVal::DONT_CARE,	state::START}},
/*DRIVE_FORWARD_SWITCH*/	{{	evtVal::IS,			evtVal::IS_NOT,		evtVal::DONT_CARE,	evtVal::DONT_CARE,	state::START}},

/*SWITCH_TO_CUBE*/			{{	evtVal::DONT_CARE,	evtVal::DONT_CARE,	evtVal::IS,			evtVal::IS_NOT,		state::SWITCH}},
/*SCALE_TO_CUBE*/			{{	evtVal::DONT_CARE,	evtVal::DONT_CARE,	evtVal::IS_NOT,		evtVal::IS,			state::SCALE}},

/*CUBE_ACROSS_SWITCH*/		{{	evtVal::IS_NOT,		evtVal::IS,			evtVal::IS_NOT,		evtVal::IS,			state::PICKUP}},
/*CUBE_TO_SWITCH*/			{{	evtVal::IS,			evtVal::IS,			evtVal::IS_NOT,		evtVal::IS,			state::PICKUP}},
/*CUBE_TO_SWITCH*/			{{	evtVal::IS_NOT,		evtVal::IS_NOT,		evtVal::IS_NOT,		evtVal::IS,			state::PICKUP}},

/*CUBE_ACROSS_SCALE*/		{{	evtVal::IS,			evtVal::IS_NOT,		evtVal::IS,			evtVal::IS_NOT,		state::PICKUP}},
/*CUBE_TO_SCALE*/			{{	evtVal::IS_NOT,		evtVal::IS_NOT,		evtVal::IS,			evtVal::IS_NOT,		state::PICKUP}}
	}};

	void stateStart();
	void stateSwitch();
	void stateScale();
	void statePickup();
	void stateMiddle();

	int returnTransition();
	int runTransition(transitions t);

// Transitions:
	int driveForwardScale();
	int driveAcrossSwitch();
	int driveForwardSwitch();
	int switchToCube();
	int scaleToCube();
	int cubeAcrossSwitch();
	int cubeToSwitch();
	int cubeAcrossScale();
	int cubeToScale();



	int centerXPolarity;
	int currentState;
	int currentTransition;
};

#endif /* SRC_STATEMACHINE_H_ */
