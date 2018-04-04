#include <iostream>
#include <string>

#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>

#include "Driving.h"
#include "Lift.h"
#include "StateMachine.h"

#define INVERT_SIDE(x) x=='R'?'L':'R'

class Robot : public frc::IterativeRobot {
private:
	frc::SendableChooser<std::string> m_chooser;
	const std::string left = "left";
	const std::string middle = "middle";
	const std::string right = "right";
	std::string m_autoSelected;
	std::string gameMessage;
	//char gameMessage[3] = {'L','L','L'};

	frc::SendableChooser<std::string> switch_chooser;
	const std::string sw_yes = "sure";
	const std::string sw_no = "nah, fam";
	std::string switch_selected;
	frc::SendableChooser<std::string> priority;
	const std::string switch_priority = "switch";
	const std::string scale_priority = "scale";

	frc::SendableChooser<std::string> cube_chooser;
	std::string try_two_cube;
	const std::string two_priority = "two cube";
	const std::string one_priority = "one cube";


	Driving *driving;
	Lift *lift;
	stateMachine *SM;
	int phase = 0;
	Joystick *joyLeft;
	Joystick *joyRight;
	Joystick *buttonBoard;

public:
	void RobotInit() {
		m_chooser.AddDefault(left, left);
		m_chooser.AddObject(middle, middle);
		m_chooser.AddObject(right, right);
		frc::SmartDashboard::PutData("Auto Modes", &m_chooser);//put the sendable chooser options in the smartdashboard

		switch_chooser.AddDefault(sw_yes, sw_yes);
		switch_chooser.AddObject(sw_no, sw_no);
		frc::SmartDashboard::PutData("Choose Switch", &switch_chooser);//put the sendable chooser options in the smartdashboard
		priority.AddObject(switch_priority, switch_priority);
		priority.AddDefault(scale_priority, scale_priority);
		frc::SmartDashboard::PutData("Priority", &priority);//put the sendable chooser options in the smartdashboard



		cube_chooser.AddObject(two_priority, two_priority);
		cube_chooser.AddDefault(one_priority, one_priority);
		frc::SmartDashboard::PutData("Two Cube?", &cube_chooser);//put the sendable chooser options in the smartdashboard


		driving = new Driving();
		lift = new Lift();
		joyLeft = new Joystick(2);
		joyRight = new Joystick(1);
		buttonBoard = new Joystick(0);
		driving->wharhs->Reset();
	}

	void AutonomousInit() {
		//driving->resetAuto();
		m_autoSelected = m_chooser.GetSelected();
		switch_selected = switch_chooser.GetSelected();
		//m_autoSelected = SmartDashboard::GetString("Auto Selector", kAutoNameDefault); get SmartDashboard data
		char initPos;

		if (m_autoSelected == middle) {
			initPos = 'M';
		} else if (m_autoSelected == right){
			initPos = 'R';
		}
		else{
			initPos = 'L';
		}
		if (SM != NULL)
			delete SM;
		gameMessage = frc::DriverStation::GetInstance().GetGameSpecificMessage();
		driving->resetEncoders();

	//	lift->reset();
phase = 0;
driving->firstTime = true;
		if (priority.GetSelected() == switch_priority)
			gameMessage[1] = gameMessage[1] == initPos ? INVERT_SIDE(gameMessage[1]) : gameMessage[1]; //?!?! You're saying that the scale is on the other side? No!!!
			//If you want to be messing with priority, you should be doing it INSIDE the state machine. That's the only way to do 2 cube auton
		SM = new stateMachine(gameMessage.c_str(), initPos, driving, lift);
		SM->timer->Start();

	}

	void AutonomousPeriodic() {
		driving->updatePosition();
//		SM->runState();
		//driving->AutoForwards(100);


		switch(-1){
		case 0:
			if(driving->AutoForwardsUniversal(300,0.0))
			{
				printf("end of phase 0\n");
				phase++;
			}
			break;
		case 1:
			if(driving->turnUniversal(90.0,1.0))  //good for 90.
			{
				printf("end of phase 1\n");
				phase++;
			}
			break;
		case 2:
			if(driving->AutoForwardsUniversal(150.0,90.0))
			{
				printf("end of phase 0\n");
				phase++;
			}
			break;
		case 3:
			if(driving->turnUniversal(-90,1.0))  //good for 90.
			{
				printf("end of phase 1\n");
				phase++;
			}
			break;
		case 4:
			if(driving->AutoForwardsUniversal(150.0,-90.0))
			{
				printf("end of phase 0\n");
				phase++;
			}
			break;
		case 5:
			if(driving->turnUniversal(-180.0,1.0))  //good for 90.
			{
				printf("end of phase 1\n");
				phase++;
			}
			break;
		case 6:
			if(driving->AutoForwardsUniversal(200,-180.0))
			{
				printf("end of phase 0\n");
				phase++;
			}
			break;
		}

		//driving->AutoTurnEnc(90);
		//driving->AutoTurnGlobal(-90);
		//driving->AutoForwards(50);
		//driving->AutoForwardsIndep(130);
		SmartDashboard::PutNumber("integral",driving->integral );
		SmartDashboard::PutNumber("rEnc", driving->pos.left);
		SmartDashboard::PutNumber("lEnc", driving->pos.right);
		SmartDashboard::PutNumber("speed", driving->speed);
		SmartDashboard::PutNumber("angle", driving->wharhs->GetAngle());
		SmartDashboard::PutNumber("int_angle", driving->integral_angle);
		SmartDashboard::PutNumber("error_angle", driving->error_angle);
		SmartDashboard::PutNumber("error forward", driving->error);
	}

	void TeleopInit()
	{
		driving->resetEncoders();
		driving->wharhs->Reset();
	//	lift->reset();
	}

	void TeleopPeriodic() {
		driving->updatePosition();
		//driving->AutoForwards(35);


		driving->tankDrive(joyLeft,joyRight);
		lift->runLift(joyLeft,joyRight,buttonBoard);
		/*switch(phase){
		case 0:
			if(driving->autoCube(lift))
				phase++;
			break;
		case 1:
			if(driving->AutoForwards(20))
				phase++;
			break;
		case 2:
			if(lift->auto_claw_clamp())
				phase++;
			break;
		}*/

//		SmartDashboard::PutNumber("claw pot", lift->clawPot->GetValue());

		//driving->AutoTurn(90);
		SmartDashboard::PutNumber("integral", driving->integral);
		SmartDashboard::PutNumber("rEnc", driving->right_encoder);
		SmartDashboard::PutNumber("lEnc", driving->left_encoder);
		SmartDashboard::PutNumber("speed", driving->speed);
		SmartDashboard::PutNumber("angle", driving->wharhs->GetAngle());
		SmartDashboard::PutNumber("int_angle", driving->integral_angle);
		SmartDashboard::PutNumber("error_angle", driving->error_angle);

		SmartDashboard::PutNumber("lift encoder", -lift->lift->GetSensorCollection().GetQuadraturePosition());


	}

	void TestPeriodic() {

	}

};

START_ROBOT_CLASS(Robot)
