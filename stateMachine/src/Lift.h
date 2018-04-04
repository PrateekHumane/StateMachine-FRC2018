/*
 * Lift.h
 *
 *  Created on: Feb 12, 2018
 *      Author: RoboWarriors
 */

#ifndef SRC_LIFT_H_
#define SRC_LIFT_H_
#include "ctre/Phoenix.h"
#include <thread>
#include <WPILib.h>

#include "states.h"
#include <Timer.h>
//#define MAX_LIFT_HEIGHT 	0.0
#define LIFT_LOW			0.0
#define LIFT_HIGH			22706		//TODO: Calculate this based on height of lift/((chain link length)*(teeth on sprocket))*(ticks per rev). NO. DO NOT CALULATE IT. THAT's the definition of stupidity.
#define LIFT_COEFFICIENT_UP		(1.0/900.0)	//coefficents for moving the lift. different for up and down so that it can fight against gravity better
#define LIFT_COEFFICIENT_DOWN	(1.0/2000.0)

#define CLAW_POT_LOW 75
//#define CLAW_POT_HIGH 1200 BUBBLES!!
#define CLAW_POT_HIGH 873
#define INTAKE_SPEED 0.35
#define SPIT_SPEED = 0.45
#define CLAW_COEFFICIENT (1.0/200.0)

#define ARM_1_PARITY -1.0
#define ARM_0_PARITY 1.0

#define LIFT_TOLERANCE 300

class Lift {
private:
	//the methods for moving the lift. see PrimaryLift
	enum LIFT_MODE {
		SLIDER = 0,
		SET = 1,
		BUTTON = 2,
		JOYSTICK = 3
	};


/*
 * 	clawLinear = new TalonSRX(9);					//claw linear x
		arm0 = new TalonSRX(3);						//intake
		arm1 = new TalonSRX(10);					//intake
		armAct = new TalonSRX(4);					//rotate claw up
		lift = new TalonSRX(8);						//lift to move vertically
 */
	TalonSRX * clawLinear;
	TalonSRX * arm0;
	TalonSRX * arm1;
	TalonSRX * armAct;

	DigitalInput * arm_lim_out;
	DigitalInput * arm_lim_in;

//	DigitalInput * arm_lim_cube_normal;
//	DigitalInput * arm_lim_cube_side;
//	Encoder * liftEncoder;
	DigitalInput * topLift;
	DigitalInput * bottomLift;


	double linearSpeed;
	void reset_internal();

	bool DONT_TOUCH_A_DE_LIFT = 0; //with the italian hand gesture
	void primaryLift(Joystick * driverstation, LIFT_MODE);
	double lift_speed_set = 1;
	double lift_speed_high = 1.0;
	double lift_speed_low = 0.5;
	int set_goal;
	void moveLift(double speed);
	void moveClaw(double speed);
	int set_claw_goal = 0;
	bool firstTimer = true;
	Timer *time;
public:
	AnalogInput * clawPot;
	bool PIDLift(int goal);
	enum SET_HEIGHTS
		{
			BOTTOM = (int)LIFT_LOW,
			EXCHANGE = 823,
			SWITCH = 7913,
			SCALE = 23000
			//ETC
		};
    enum CLAW_MODE {
        SET_CLAW = 0,
        BUTTON_CLAW = 1
    };
    enum CLAW_SETS {
        REGULAR_OPEN = 357,//428
        REGULAR_CLOSED = 428,
        SIDE_OPEN = 772,
        SIDE_CLOSED =850
    };


   Encoder * liftEnc;


	Lift();
	virtual ~Lift();
	bool auto_claw_open();
	bool auto_claw_clamp();
	void claw_set(int set);
	void runClaw(double speed);
	bool auto_lift(SET_HEIGHTS set);
	void reset();
	void primaryClaw(Joystick * left, Joystick * right, Joystick * driverstation, CLAW_MODE mode);
	void runLift(Joystick * left, Joystick * right, Joystick * driverstation);
	TalonSRX * lift;
	TalonSRX * lift0;

};

#endif /* SRC_LIFT_H_ */
