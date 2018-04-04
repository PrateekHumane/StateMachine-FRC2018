/*
 * Lift.cpp
 *
 *  Created on: Feb 12, 2018
 *      Author: RoboWarriors
 */

#include "Lift.h"
#include <WPILib.h>
#include "ctre/Phoenix.h"

Lift::Lift() {
	clawLinear = new TalonSRX(9);					//claw linear x
	arm0 = new TalonSRX(7);							//intake
	arm1 = new TalonSRX(3);						//intake DOMINIQUE IS 3!!!! nubbles 7
	armAct = new TalonSRX(4);						//rotate claw up
	lift = new TalonSRX(8);							//lift to move vertically. ALSO THE LIFT ENCODER
	lift0 = new TalonSRX(10);

	liftEnc = new Encoder(9,8);

	arm_lim_out = new DigitalInput(2);//limti switch
	arm_lim_in = new DigitalInput(3);
	topLift = new DigitalInput(0);
	bottomLift = new DigitalInput(1);
	//limit switches are plugged into Signal and Ground. They are 1 when open and 0 when closed.
	//liftEncoder = new Encoder(8,9); // lift encoder is now on a talon.
	linearSpeed = 0;
	set_goal = 0;
	time = new Timer();
	clawPot = new AnalogInput(0);//potentiometer
}

Lift::~Lift() {};	//who even uses destructors.

void Lift::moveLift(double speed) // for safty reasons.
{
	//If it’s going up and hasn’t reached the top, or it’s going down and hasn’t reached the bottom
	//if( (speed < 0 && topLift->Get()) || (speed > 0 && bottomLift->Get())

	SmartDashboard::PutBoolean("top",topLift->Get());
	SmartDashboard::PutBoolean("bottom",bottomLift->Get());
	SmartDashboard::PutNumber("lift encoder",lift->GetSensorCollection().GetQuadraturePosition());


	//printf("%f\n",speed);
	//stop it with the limit
/*
	if(!topLift->Get())
	{
		//if it is hitting the top and going up, stop
		if(speed > 0.0)
			speed = 0.0;
	}
	if(!bottomLift->Get())
	{
		//if its hitting the bottom and going down, stop
		if(speed < 0.0)
			speed = 0.0;
	}
*/

	lift->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,  speed);
	lift0->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,  speed);
}

void Lift::moveClaw(double speed) // for safty reasons.
{
	//If it’s going up and hasn’t reached the top, or it’s going down and hasn’t reached the bottom
	//if( (speed < 0 && topLift->Get()) || (speed > 0 && bottomLift->Get())

	SmartDashboard::PutBoolean("out",arm_lim_out->Get());
	SmartDashboard::PutBoolean("in",arm_lim_in->Get());


	if(clawPot->GetValue() <= CLAW_POT_LOW)//!arm_lim_out->Get())
	{
		//is hitting.
		if(speed < 0.0)
			speed = 0.0;
	}

 	if(clawPot->GetValue() >= CLAW_POT_HIGH)//!arm_lim_in->Get())
	{
		//is hitting.
		if(speed > 0.0)
			speed = 0.0;
	}

	//lift->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,  lift_speed);

	//if( ( speed < 0 && !bottomLift->Get()) || (speed > 0 && !topLift->Get()))
		//lift->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);
	//else
	clawLinear->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,  speed);
	SmartDashboard::PutNumber("clamp speed", speed);
}

//this is thread'd out to allow it to run in a while loop without breaking the regular code.
//it resets the encoder so it is zero at the bottom.
void Lift::reset_internal()
{
	//HAVE IT MOVE UP FIRST FOR LIKE 500ms, then go down.
	arm0->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, ARM_0_PARITY*1.0);
	arm1->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, ARM_1_PARITY*1.0);
	moveLift(1.0);
	Wait(0.5);
	while(1)
	{
		if(!bottomLift->Get())
		{
			//it is at the bottom
			lift->GetSensorCollection().SetQuadraturePosition(0,500);//reset
			liftEnc->Reset();
			arm0->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
			arm1->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
			moveLift(0.0);//stop moving
			//printf("done");
			break;//exit loop.
		}
		else
		{
			moveLift(-0.45);	//if not at the bottom, go down.
			//printf("going down \n");
		}
	}
	DONT_TOUCH_A_DE_LIFT = 0;	//allow other code to touch the lift.
}

//this is the main RESET function for the lift. it makes the thread to lower the lift.
void Lift::reset()
{
	//create a thread to reset the lift.
	DONT_TOUCH_A_DE_LIFT = 1;	//this is a thing to block other code from messing with the lift while it is being reset.
	std::thread l_th(&Lift::reset_internal,this); //thread stuff
	l_th.detach();
//	lift->GetSensorCollection().SetQuadraturePosition(0,500);//reset
//	DONT_TOUCH_A_DE_LIFT = 0;	//allow other code to touch the lift.


}

//runLift is the main lift function. it has all the stuff for the lift, claw and intake.
void Lift::runLift(Joystick * left, Joystick * right, Joystick * driverstation)
{
	if(driverstation != NULL)
	{

		//Peter's code is *not* well-documented
		//CLAW INTAKE ===========================
		if(USE_CONTROLLER)
		{
			if (driverstation->GetRawButton(3)){
				//Rotate cube left
				SmartDashboard::PutString("Claw Status", "Rotate cube left");
				arm0->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,  INTAKE_SPEED);
				arm1->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, INTAKE_SPEED);
			} else if (driverstation->GetRawButton(2)){
				//Rotate cube right
				SmartDashboard::PutString("Claw Status", "Rotate cube right");
				arm0->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,  -INTAKE_SPEED);
				arm1->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -INTAKE_SPEED);
			} else if (driverstation->GetRawButton(4)){
				//Push cube out
				SmartDashboard::PutString("Claw Status", "Push cube out");
				arm0->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,  -INTAKE_SPEED);
				arm1->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, INTAKE_SPEED);

			} else if (driverstation->GetRawButton(1)){
				//Pull cube in
				SmartDashboard::PutString("Claw Status", "Pull cube in");
				arm0->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, INTAKE_SPEED);
				arm1->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -INTAKE_SPEED);
			} else {
				//Stop motors
				SmartDashboard::PutString("Claw Status", "Stop claw motors");
				arm0->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,  0.0);
				arm1->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);
			}

		}
		else
		{
			double intake_speed = ((-driverstation->GetRawAxis(BUTTON_BOARD::SLIDER_4) + 1.0)/2.0)*(1-INTAKE_SPEED)+INTAKE_SPEED;

			if (driverstation->GetRawButton(BUTTON_BOARD::BUTTON_LEFT_LEFT)){
				//Rotate cube left
				SmartDashboard::PutString("Claw Status", "Rotate cube left");
				arm0->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,  -ARM_0_PARITY  *intake_speed);
				arm1->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,ARM_1_PARITY *  intake_speed);
			} else if (driverstation->GetRawButton(BUTTON_BOARD::BUTTON_LEFT_RIGHT)){
				//Rotate cube right
				SmartDashboard::PutString("Claw Status", "Rotate cube right");
				arm0->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,  ARM_0_PARITY  *intake_speed);
				arm1->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -ARM_1_PARITY * intake_speed);
			} else if (driverstation->GetRawButton(BUTTON_BOARD::BUTTON_LEFT_UP) && driverstation->GetRawButton(BUTTON_BOARD::ROCKER_2_UP)){
				//Push cube out
				SmartDashboard::PutString("Claw Status", "Push cube out");
				arm0->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,  -ARM_0_PARITY  *intake_speed);
				arm1->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -ARM_1_PARITY * intake_speed);

			}  else if (driverstation->GetRawButton(BUTTON_BOARD::BUTTON_LEFT_UP)){
				//Push cube out
				SmartDashboard::PutString("Claw Status", "Push cube out");
				arm0->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,  -ARM_0_PARITY  * intake_speed);
				arm1->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -ARM_1_PARITY * intake_speed);

			} else if (driverstation->GetRawButton(BUTTON_BOARD::BUTTON_LEFT_DOWN) || left->GetRawButton(1)){
				//Pull cube in
				SmartDashboard::PutString("Claw Status", "Pull cube in");
				arm0->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, ARM_0_PARITY  * intake_speed);
				arm1->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, ARM_1_PARITY * intake_speed);
			} else {
				//Stop motors
				SmartDashboard::PutString("Claw Status", "Stop claw motors");
				arm0->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,  0.0);
				arm1->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);
			}
		}



		if(driverstation->GetRawButton(BUTTON_BOARD::TOGGLE_RIGHT))
		{
			//over-rided (over ridden?)
			SmartDashboard::PutString("clamp goal", "toggle on but really off");

			if(driverstation->GetRawButton(BUTTON_BOARD::BUTTON_RIGHT_UP) || driverstation->GetRawButton(BUTTON_BOARD::BUTTON_RIGHT_RIGHT) || driverstation->GetRawButton(BUTTON_BOARD::BUTTON_RIGHT_LEFT) || driverstation->GetRawButton(BUTTON_BOARD::BUTTON_RIGHT_DOWN))
			{
				//then its pressing
				primaryClaw(left, right, driverstation, CLAW_MODE::SET_CLAW);
			}
			else
			{
				primaryClaw(left, right, driverstation, CLAW_MODE::BUTTON_CLAW);
			}
		}
		else
		{
			if(right->GetRawButton(1))
			{
				SmartDashboard::PutString("clamp goal", "open");
				claw_set(CLAW_POT_LOW);
			}
			else
			{
				SmartDashboard::PutString("clamp goal", "closed");
				claw_set(CLAW_SETS::REGULAR_CLOSED);
			}
		}










/*
		//CLAW 	=============================================
		//to move the claw in an out.
		// (+) is in; (-) is out.
		//it is so that if either the first or second driver touches it. goes.
		if(driverstation->GetRawButton(BUTTON_BOARD::ROCKER_1_DOWN)||left->GetRawButton(1))
		{
			//move in
			moveClaw(1.0);
		}
		else if (driverstation->GetRawButton(BUTTON_BOARD::ROCKER_1_UP)||right->GetRawButton(1))
		{
			//move out
			moveClaw(-1.0);
		}
		else
		{
			//dont move
			moveClaw(0.0);
		}
*/


		//LIFT	============================================
		//if any of the set heights are pushed, then go to set heights.
		// ROCKER 5 IS BROKEEEEEN
		if(/*driverstation->GetRawButton(BUTTON_BOARD::ROCKER_5) || */driverstation->GetRawButton(BUTTON_BOARD::ROCKER_6) || driverstation->GetRawButton(BUTTON_BOARD::ROCKER_7) || driverstation->GetRawButton(BUTTON_BOARD::ROCKER_8))
		{
			//go to set heights
			primaryLift(driverstation, LIFT_MODE::SET);
		}
		else
		{
			//if none of the sets are being pressed, the use the other methods.
			//there is a toggle to chose SLIDER or BUTTON move methods
			if(USE_CONTROLLER)
			{
				primaryLift(driverstation,LIFT_MODE::JOYSTICK);
			}
			else
			{
				if(driverstation->GetRawButton(BUTTON_BOARD::TOGGLE_LEFT))
				{
					//lift with BUTTON Mode
					primaryLift(driverstation, LIFT_MODE::BUTTON);
				}
				else
				{
					//lift with SLIDER mode
					primaryLift(driverstation, LIFT_MODE::SLIDER);
				}
			}
		}




		//this all is garbage

		//primaryLift(driverstation, LIFT_MODE::BUTTON);
		/*		if( driverstation->GetRawButton(BUTTON_BOARD::ROCKER_1_DOWN) )
		{
			arm0->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,  INTAKE_SPEED);
			arm1->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -INTAKE_SPEED);
		}
		else if ( driverstation->GetRawButton(BUTTON_BOARD::ROCKER_1_UP) )
		{
			arm0->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,  INTAKE_SPEED);
			arm1->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -INTAKE_SPEED);
		}
/*
		if(right->GetRawButton(1) == left->GetRawButton(1))
			clawLinear->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);
		else if(right->GetRawButton(1))
			clawLinear->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 1.0);
		else
			clawLinear->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -1.0);
*/
		//drv->tankDrive(controller->GetRawAxis(0));
	//	double dinodan = enc->Get() * 2.8125;
	//	SmartDashboard::PutNumber("enc",dinodan);
		//possibly add a bias to keep the arm up. something like +0.2
	//	clawLinear->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -driverstation->GetRawAxis(0)*0.5);
	//	SmartDashboard::PutNumber("right encoder",rightEncoder->GetSensorCollection().GetQuadraturePosition()*(360.0/1024.0));
	//	SmartDashboard::PutNumber("left encoder",leftEncoder->GetSensorCollection().GetQuadraturePosition()*(360.0/1024.0));
		//SmartDashboard::PutNumber("that number",controller->GetRawAxis(5));
		/*if(joyRight->GetRawButton(1))
			armAct->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 1.0);
		if (joyLeft->GetRawButton(1))
			armAct->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -1.0);
		else
			armAct->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);/**
		int pov = driverstation->GetPOV(0);
		linearSpeed = 0;
		double deploySpeed = 0;
		if (pov != -1)			//POV is pressed
		{

			if (pov <= 135 && pov >=45)							//open claw
				linearSpeed = 1;
			if (pov <= 315 && pov >=225)						//close claw
				linearSpeed = -1;
			if (pov == 0 || pov == 315 || pov == 45)			//stow claw
				deploySpeed = 1;
			if (pov == 180 || pov == 135 || pov == 225)			//deploy claw
				deploySpeed = -1;
		}
/*
		if( (linearSpeed > 0 && arm_lim_out->Get()) || (linearSpeed < 0 && arm_lim_in->Get()) )
			clawLinear->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, linearSpeed);
		else
			clawLinear->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);
/**
		armAct->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, deploySpeed);

		if(driverstation->GetRawButton(1))
		{
			arm0->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,  driverstation->GetRawAxis(3));
			arm1->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -driverstation->GetRawAxis(3));
		}
		else
		{
			arm0->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -driverstation->GetRawAxis(3));
			arm1->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,  driverstation->GetRawAxis(3));
		}


		if(driverstation->GetRawButton(BUTTON_BOARD::ROCKER_5) || driverstation->GetRawButton(BUTTON_BOARD::ROCKER_6) || driverstation->GetRawButton(BUTTON_BOARD::ROCKER_7) || driverstation->GetRawButton(BUTTON_BOARD::ROCKER_8))
			primaryLift(driverstation, LIFT_MODE::SET);
		else
			primaryLift(driverstation, LIFT_MODE::SLIDER);
	*/
	}
}

//PID Lift move the lift to a given height based on the input and the encoder. it requires the lift to be RESET prior.
//
bool Lift::PIDLift (int goal)
{
	//the fun slider one
	//slipry
	//making the lifty thing do stuff with a slidy boi.
	//										edit: u/prateek boy->boi
	// soodo cod:
	//this will be obtained experimentally. This is the encoder value of the highest position zeroed at the bottom.
	//make it move to a position based on Drive station sliders. assume there is a joystick named driver_station. the slider is Axis 0 [-1,1]
	//double current = lift->GetSensorCollection().GetQuadraturePosition(); // assume it has been zero'd
	//Gets joystick input


	//get the current height
	double current = -lift->GetSensorCollection().GetQuadraturePosition();
	//gets the error to determine speed
	double error = goal - current;

	if(fabs(error) < LIFT_TOLERANCE)
	{
		//return true. stop
		moveLift(0.0);
		return true;
	}

	//printf("lifting\n");
	//We use different coefficients because it goes down hella fast and up slowly. so we have a higher coefficient going up. and down slower.
	if(error > 0.0)
	{
		//printf("going up?\n");
		//if it is going up, use the up coefficient
		moveLift(LIFT_COEFFICIENT_UP * error);
	}
	else
	{
		double err = LIFT_COEFFICIENT_DOWN * error;
		if(err < -0.45)
			err = -0.45;
		//if it is going down, use the down coefficient
		moveLift(err);
	}
	return false;
}

//this is the code to run the lift part.
void Lift::primaryLift(Joystick * driverstation, LIFT_MODE mode)
{
	/*
	 * 	if(!DONT_TOUCH_A_DE_LIFT)
			lift->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,  lift_speed);


		//slipry
		//making the lifty thing do stuff with a slidy boi.
		//										edit: u/prateek boy->boi
		// soodo cod:
		//#define LIFT_LOW 0.0
		//#define LIFT_HIGH xxxxxxx.xxxxxx			//this will be obtained experimentally. This is the encoder value of the highest position zeroed at the bottom.
			//make it move to a position based on Drive station sliders. assume there is a joystick named driver_station. the slider is Axis 0 [-1,1]

		//	SmartDashboard::PutNumber("the lift is at....",liftEncoder->Get());

		//	SmartDashboard::PutBoolean("that limit",topLift->Get());
		//	printf("limit %d\n",topLift->Get());
	 */
	if(!DONT_TOUCH_A_DE_LIFT)		//if the lift is reseting, player has no control over it.
	{
		//slider mode
		if(mode == LIFT_MODE::SLIDER)
		{
			double slider = -driverstation->GetRawAxis(BUTTON_BOARD::SLIDER_3); // -1 to 1
			//Maps the slider value from 0 to 1 and multiplies by LIFT_HIGH
			double goal = ((slider + 1.0)/2.0)*(LIFT_HIGH-LIFT_LOW) + LIFT_LOW;
			PIDLift(goal);//runs the PIDlift to move to the slider value
			//printf("slider\n");

		}
		else if(mode == LIFT_MODE::BUTTON)
		{
			//button based
			//uses buttons to move up and down
			//starts the speed at zero
			double lift_speed = 0.0;
			lift_speed_set = (((-driverstation->GetRawAxis(BUTTON_BOARD::SLIDER_2)) + 1.0)/2.0)*(lift_speed_high-lift_speed_low) + lift_speed_low;
			//if any buttons are pressed, then set the speed, otherwise it doesn't move.
			if(driverstation->GetRawButton(BUTTON_BOARD::ROCKER_4_UP))
			{
				//move up
				lift_speed = lift_speed_set;
			}
			else if (driverstation->GetRawButton(BUTTON_BOARD::ROCKER_4_DOWN))
			{
				//move down
				lift_speed = -lift_speed_set;
//				if(lift->GetSensorCollection().GetQuadraturePosition() < LIFT_HIGH/4)
//				{
					//this is to stop it from slamming into the ground.
					//if it is in the bottom quarter, slow it down.
					//lift_speed *= 0.5;
//				}
			}
			//move the lift
			//printf("lift speed: %f \n", lift_speed);
			moveLift(lift_speed);
		}
		else if (mode == LIFT_MODE::SET)
		{
			//printf("setting\n");
			//goes to set height
			//map buttons to chose the height
			//if(driverstation->GetRawButton(N))
			//{
			//	set_goal = SET_HEIGHTS::SCALE;
			//}
			// its necessary to call the PIDLift() every loop. so that it keeps moving.
/*			if(driverstation->GetRawButton(ROCKER_5))
			{
				set_goal = SET_HEIGHTS::BOTTOM;
			}*/
			if(driverstation->GetRawButton(ROCKER_6))
			{
				set_goal = SET_HEIGHTS::BOTTOM;//EXCHANGE WHEN ROCKER 5 IS FIXED
			}
			if(driverstation->GetRawButton(ROCKER_7))
			{
				set_goal = SET_HEIGHTS::SWITCH;
			}
			if(driverstation->GetRawButton(ROCKER_8))
			{
				set_goal = SET_HEIGHTS::SCALE;
			}
			PIDLift(set_goal);
			//auto_lift((SET_HEIGHTS)set_goal);
		}
		else if(mode == LIFT_MODE::JOYSTICK)
		{
			//uses a joystick. Similar to BUTTON. but for the XBOX controller
			double lift_speed = -driverstation->GetRawAxis(1);
			moveLift(lift_speed);
		}
	}
}

bool Lift::auto_lift(SET_HEIGHTS set)
{
	/*

	if(fabs(-lift->GetSensorCollection().GetQuadraturePosition() - set) < LIFT_TOLERANCE)
	{
		//return true. stop
		moveLift(0.0);
		return true;
	}
	else
	{
		if()


		PIDLift(set);
	}
	return false;
	*/
	int error = -lift->GetSensorCollection().GetQuadraturePosition() - set;
	if(fabs(error) < LIFT_TOLERANCE)
	{
		//return true. stop
		moveLift(0.0);
		return true;
	}
	else
	{
		if(error < 0.0)
		{
			moveLift(-0.05);
		}
		else
		{
			moveLift(0.5);
		}


	//	PIDLift(set);
	}
	return false;



}

void Lift::primaryClaw(Joystick * left, Joystick * right, Joystick * driverstation, CLAW_MODE mode)
{
	SmartDashboard::PutNumber("that dank claw", clawPot->GetValue());
     if(mode == CLAW_MODE::BUTTON_CLAW)
    {
         if(driverstation->GetRawButton(BUTTON_BOARD::ROCKER_1_DOWN))//||left->GetRawButton(1))
         {
             //move in
             moveClaw(1.0);
         }
         else if (driverstation->GetRawButton(BUTTON_BOARD::ROCKER_1_UP))//)||right->GetRawButton(1))
         {
             //move out
             moveClaw(-1.0);
         }
         else
         {
             //dont move
             moveClaw(0.0);
         }
    }
    else if (mode == CLAW_MODE::SET_CLAW)
    {
        //BUTTON_RIGHT_UP is regular open
        //BUTTON_RIGHT_RIGHT is regular closed
        //BUTTON_RIGHT_LEFT is sideways open
        //BUTTON_RIGHT_DOWN is sideways closed
        if(driverstation->GetRawButton(BUTTON_RIGHT_UP))
        {
            set_claw_goal = CLAW_SETS::REGULAR_OPEN;
        }
        if(driverstation->GetRawButton(BUTTON_RIGHT_RIGHT))
        {
            set_claw_goal = CLAW_SETS::REGULAR_CLOSED;
        }
        if(driverstation->GetRawButton(BUTTON_RIGHT_LEFT))
        {
            set_claw_goal = CLAW_SETS::SIDE_OPEN;
        }
        if(driverstation->GetRawButton(BUTTON_RIGHT_DOWN))
        {
            set_claw_goal = CLAW_SETS::SIDE_CLOSED;
        }
        claw_set(set_claw_goal);
    }
}


void Lift::claw_set(int set)
{
	SmartDashboard::PutString("clamp status", "claw_set");
	  double current = clawPot->GetValue();
	//gets the error to determine speed
	double error = (set - current);
	SmartDashboard::PutNumber("clamp error", error);
	moveClaw(CLAW_COEFFICIENT * error);
}



bool Lift::auto_claw_open()
{
	/*if(clawPot->GetValue() <= CLAW_SETS::REGULAR_OPEN)//<= 72)//!arm_lim_out->Get())
	{
		arm0->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,  0.0);
		arm1->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);
		//return true. stop
		moveClaw(0.0);
		return true;
	}
	else
	{*/
	if(firstTimer){
		time->Reset();
		time->Start();
		firstTimer = false;
	}


	arm0->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -ARM_0_PARITY * 1);
	arm1->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -ARM_1_PARITY * 1);
	if(time->Get() < 1.0)
	{
		return false;
	}
	firstTimer = true;
	arm0->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);
	arm1->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);
	//firstTimer = true; -- ADD THIS IN LATER
/*		moveClaw(-1.0);
	}*/
	return true;
}

bool Lift::auto_claw_clamp()
{
	if(clawPot->GetValue() >= CLAW_SETS::REGULAR_CLOSED)//!arm_lim_out->Get())
	{
		arm0->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,  0.0);
		arm1->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);
		//return true. stop
		moveClaw(0.0);
		return true;
	}
	else
	{

		arm0->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -ARM_0_PARITY * 0.5);
		arm1->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, ARM_1_PARITY * 0.5);

		moveClaw(1.0);
	}
	return false;
}

void Lift::runClaw(double speed)
{
	arm0->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, speed);
	arm1->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -speed);
}
