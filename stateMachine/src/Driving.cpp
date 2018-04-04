/*
 * tankdrivetest.cpp
 *
 *  Created on: Jan 12, 2018
 *      Author: RoboWarriors
 */

#include <ctre/phoenix/ErrorCode.h>
#include <ctre/phoenix/MotorControl/CAN/TalonSRX.h>
#include <ctre/phoenix/MotorControl/ControlMode.h>
#include <ctre/phoenix/MotorControl/SensorCollection.h>
#include <ctre/phoenix/MotorControl/StatusFrame.h>
#include <Driving.h>
#include <Joystick.h>
#include <SerialPort.h>
#include <SmartDashboard/SmartDashboard.h>
#include <cmath>
#include <cstdio>

Driving::Driving() {
	leftTalon0 = new TalonSRX(TALON::LEFT_FRONT);
	leftTalon1 = new TalonSRX(TALON::LEFT_BACK);
	rightTalon0 = new TalonSRX(TALON::RIGHT_FRONT);
	rightTalon1 = new TalonSRX(TALON::RIGHT_BACK);
	rt_encoder_talon = rightTalon0;
	lf_encoder_talon = leftTalon0;
	lf_encoder_talon->SetStatusFramePeriod(StatusFrameEnhanced::Status_3_Quadrature,20,500);
	rt_encoder_talon->SetStatusFramePeriod(StatusFrameEnhanced::Status_3_Quadrature,20,500);

	//THESE ARE IMPORTANT> UNCOMMENT
	wharhs = new  AHRS(SerialPort::Port::kUSB1); //This is the USB connection
	//wharhs = new  AHRS(I2C::Port::kOnboard); //Is this I2C?
	wharhs->Reset();
	resetAuto();

	//turbo = false;
	//turbo_toggle = false;
	//leftTalon0 = new CANTalon(0);//these IDs have to be changed occasionally using the roborio-41-frc.local thing
	//rightTalon0 = new CANTalon(2);
	//leftTalon1 = new CANTalon(3);
	//rightTalon1 = new CANTalon(1);
}

void Driving::drive(double leftspeed, double rightspeed)
{
	leftTalon0->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -leftspeed);
	rightTalon0->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, rightspeed);
	leftTalon1->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -leftspeed);
	rightTalon1->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, rightspeed);
}

double signum(double a)
{
	if(a < 0)
		return -1.0;
	return 1.0;
}

double clamp(double val, double low, double high)
{
	if(val > high)
		return high;
	if(val <low)
		return low;
	return val;
}

double magnitude (double a, double b)
{
	return sqrt(a*a+b*b);
}

void Driving::resetPosition()
{
	pos.reset();
}

double wrap(double a, double bounds)
{
	if(a < 0)
		return bounds + a;
	if(a >= bounds)
		return a - bounds;
	return a;
}

void Driving::updatePosition()
{
	/*left_n = lf_encoder_talon->GetSensorCollection().GetQuadraturePosition()*(PI/180.0)*WHEEL_RADIUS/1024.0;		//say what it do
	right_n = rt_encoder_talon->GetSensorCollection().GetQuadraturePosition()*(PI/180.0)*WHEEL_RADIUS/1024.0;
	left_d = left_n - left_o;
	right_d = right_n - right_o;
	left_o = left_n;
	right_o = right_n;
	if (fabs(left_d - right_d) < 1.0e-6) {
		pos.x += left_d * cos(pos.theta);
		pos.y += right_d * sin(pos.theta);
	    //pos.theta = pos.theta;
	} else {
	    float R = AXLE_LENGHT * (left_d + right_d) / (2 * (right_d - left_d));
	    float wd = (right_d - left_d) / AXLE_LENGHT;

	    pos.x += R * sin(wd + pos.theta) - R * sin(pos.theta);
	    pos.y -= R * cos(wd + pos.theta) + R * cos(pos.theta);
	    pos.theta = wrap(pos.theta + wd, 2 * PI); // forces it to be [0,2PI)
	}
*/
	left_encoder = lf_encoder_talon->GetSensorCollection().GetQuadraturePosition();
	right_encoder = rt_encoder_talon->GetSensorCollection().GetQuadraturePosition();

	pos.left = (-lf_encoder_talon->GetSensorCollection().GetQuadraturePosition() / 4096.0) * ( PI * WHEEL_RADIUS * 2);   // TODO: DOMINQUE NEEDS TO BE NEGATIVE HERE I BELIEVE
	pos.right = (rt_encoder_talon->GetSensorCollection().GetQuadraturePosition() / 4096.0) * ( PI * WHEEL_RADIUS * 2);

	//printf("encoder left: %f encoder right: %f",left_encoder, right_encoder);

	///23.5 5.75
}

/*void Driving::tankDrive(Joystick *joyLeft,Joystick *joyRight) {

	if(joyLeft->GetRawButtonPressed(1))		//toggles between turbo and normal speed modes
	{
		if(!turbo_toggle)
		{
			turbo = !turbo;
		}
		turbo_toggle = 1;
	}
	else
	{
		turbo_toggle = 0;
	}


	//throttle = -1.0*clamp(joy1->GetRawAxis(1),-1.0,0.0)*(max_throt - min_throt) + min_throt;




	move(joyLeft->GetRawAxis(1), joyRight->GetRawAxis(1));

	//move(joyRight->GetRawAxis(1) - joyRight->GetRawAxis(0) , joyRight->GetRawAxis(1) + joyRight->GetRawAxis(0));


	if(turbo)
	{
		//	move(joy1->GetRawAxis(1)*0.6, joy2->GetRawAxis(1)*0.6);
	}
	else
	{
		//	move(joy1->GetRawAxis(1)*0.6, joy2->GetRawAxis(1)*0.6);
		//move(joy1->GetRawAxis(1)/4, joy2->GetRawAxis(1)/4);
		//move(joy1->GetRawAxis(1)/4, joy1->GetRawAxis(5)/4);
	}

}
*/

void Driving::tankDrive(Joystick *joyLeft,Joystick *joyRight, Joystick *buttonBoard)
{
	if(buttonBoard != NULL){
		//stuff with button board control...
		//maybe speed.
	}

	//throttle = -1.0*clamp(joy1->GetRawAxis(1),-1.0,0.0)*(max_throt - min_throt) + min_throt;

	SmartDashboard::PutNumber("RIGHT ENCODER VALUE",rt_encoder_talon->GetSensorCollection().GetQuadraturePosition());
	SmartDashboard::PutNumber("LEFT ENCODER VALUE",lf_encoder_talon->GetSensorCollection().GetQuadraturePosition());

	SmartDashboard::PutNumber("leftspeed",joyLeft->GetRawAxis(1));
	SmartDashboard::PutNumber("rightspeed",joyRight->GetRawAxis(1));

//	SmartDashboard::PutNumber("left Joystick",joyLeft->GetRawAxis(1));
//	SmartDashboard::PutNumber("right Joystick",joyRight->GetRawAxis(1));


	drive(joyLeft->GetRawAxis(1), joyRight->GetRawAxis(1)/*-0.09448818862438202*/);////the constant is a fudge factor to get it to stop going backwards.
}

void Driving::ControllerMove(Joystick * controller)
{
	drive(controller->GetRawAxis(1)*0.4 - controller->GetRawAxis(0)*0.4, controller->GetRawAxis(1)*0.4 + controller->GetRawAxis(0)*0.4);
}

void Driving::setEncoders()
{
	right_encoder = rt_encoder_talon->GetSelectedSensorPosition(0);// rightTalon0->GetSelectedSensorPosition(0);
	left_encoder = lf_encoder_talon->GetSelectedSensorPosition(0);//leftTalon0->GetSelectedSensorPosition(0);
	leftTalon0->GetSensorCollection().GetQuadraturePosition(); // this is also a way to access the encoder.
}

void Driving::resetEncoders()
{
	rt_encoder_talon->GetSensorCollection().SetQuadraturePosition(0,500);
	lf_encoder_talon->GetSensorCollection().SetQuadraturePosition(0,500);
//	rt_encoder_talon->SetSelectedSensorPosition(0,0,500);
//	lf_encoder_talon->SetSelectedSensorPosition(0,0,500);
//	rightTalon0->GetSelectedSensorPosition(0);
//	leftTalon0->GetSelectedSensorPosition(0);
//	leftTalon0->GetSensorCollection().GetQuadraturePosition(); // this is also a way to access the encoder.
}

/*bool Driving::AutoForwards(double goal)
{
	int polarity = -signum(goal);
	goal = fabs(goal);
	double k_p = 1.0/50;
	current = ((fabs(pos.x)+fabs(pos.y))*1.04)/2.0;
	//printf("current %f\n", current);
	error = goal - current;
	speed = error*k_p;
	integral += speed*k_i;
	if(integral > 0.4)
		integral = 0.4;
	//	drive(speed * 0.4, speed * 0.4);

	//speed is higher than 1. it equals max_speed;
	//speed is less than 1, it equals speed * mx_speed;


	if(fabs(error) < TOLERANCE)
	{
		printf("error: %f\n",error);
		drive(0.0, 0.0);
		return true;
	}
	else if (speed > 1.0){
		drive(0.5, 0.5);
	}
	else
	{
		drive(polarity*(speed*0.5+(integral)), polarity*(speed*0.5+(integral)));
	}

	return false;
}*/

/*bool Driving::AutoTurn(double angle){

	current_angle= wharhs->GetAngle();
	error_angle = fabs(angle - current_angle);
//	int polarity = signnum (error_angle);
	if(error_angle < 0.5){
		drive(0.0,0.0);
		return true;
	}
	error_angle /= 90.0;
	integral_angle +=  error_angle;
	drive(((k_pAngle*-error_angle)+(k_iAngle*-integral_angle)+(-(previous_angle-current_angle)*k_dAngle))*signum(angle),((k_pAngle*error_angle)+(k_iAngle*integral_angle)+(-(previous_angle-current_angle)*k_dAngle))*signum(angle));
	previous_angle = current_angle;

	return false;
}*/
//----- WITH OSCILLATION------//


bool Driving::AutoTurn(double angle, double tolerance){
	if (firstTime) {
		resetAuto();
		firstTime = false;
		wharhs->Reset();
		//reset global angle
	}
	current_angle= wharhs->GetAngle();
	error_angle = angle - current_angle;
//	int polarity = signnum (error_angle);
	if(fabs(error_angle) < tolerance || fabs(current_angle) > fabs(angle)){
		drive(0.0,0.0);
		firstTime = true;
		return true;
	}
	error_angle /= 90.0;
	integral_angle +=  error_angle;
	if(integral_angle > 45)// cap integral
		integral_angle = 45;
	else if(integral_angle < -45)
		integral_angle = -45;
	double speed = ((k_pAngle*-error_angle)+(k_iAngle*-integral_angle)+(-(previous_angle-current_angle)*k_dAngle));
	if (speed > 1)
		speed = 1;
	else if (speed < -1)
		speed = -1;

//	printf("speed: %f\n",speed);
	drive(speed,-speed);
	previous_angle = current_angle;

	return false;
}

bool Driving::AutoTurnEnc(double angle, double tolerance){
	if (firstTime) {
		if (resetAuto()){
			firstTime = false;
			printf("pos left: %f\n",pos.left);
		} else {
			return false;
		}
	}
	SmartDashboard::PutNumber("pos left", pos.left);
	SmartDashboard::PutNumber("pos right", pos.right);

	current_angle = (((fabs(pos.left)+fabs(pos.right)))*1.04)/2.0; //Distance traveled in inches = average of absolute value of both encoders
	current_angle *= signum(angle); //Correct for whether it is left or right turn
	printf("current angle: %f\n",current_angle);
	float goal = (WHEELBASE * PI) * (angle / 360); //Robot diameter (inches) * fraction of a whole turn
	SmartDashboard::PutNumber("encTurn current_angle", current_angle);
	SmartDashboard::PutNumber("encTurn goal", goal);
	//printf("current_angle %f\n", current_angle);
	error = goal - current_angle;
	speed = -error*k_pAngle;
	integral_angle += -error*k_iAngle;
	if(integral_angle > 0.2)
		integral_angle = 0.2;
	if(integral_angle < -0.2)
		integral_angle = -0.2;
	SmartDashboard::PutNumber("encTurn integral", integral_angle);

	//	drive(speed * 0.4, speed * 0.4);
	//speed is higher than 1. it equals max_speed;
	//speed is less than 1, it equals speed * mx_speed;


	if(fabs(current_angle) > fabs(goal))//fabs(error) < TOLERANCE)
	{
		printf("error: %f\n",error);
		drive(0.0, 0.0);
		firstTime = true;
		prev_encoders = left_encoder;
		return true;
	}
	else if (speed > 0.75){
		drive(0.75, -0.75);
	}
	else if (speed < -0.75){
		drive(-0.75, 0.75);
	}
	else
	{
		drive((speed*0.4+(integral_angle)-(current_angle-previous_angle)*k_dAngle), -(speed*0.4+(integral_angle)-(current_angle-previous_angle)*k_dAngle));
	}

	previous_angle = current_angle;

	return false;
}


bool Driving::AutoTurnGlobal(double angle, double tolerance){
	if (firstTime) {
		resetAuto();	//check in here to make sure you don't
		integral_angle = 0;
		firstTime = false;
	}
	current_angle = wharhs->GetAngle()-angleInitial;
	error_angle = angle - current_angle;
	SmartDashboard::PutNumber("error angle!:", error_angle);
	if(fabs(error_angle) < tolerance || fabs(current_angle) > fabs(angle-tolerance)){
		drive(0.0,0.0);
		firstTime = true;
		return true;
	}
	error_angle /= 90.0;
	integral_angle +=  error_angle;
	if(integral_angle > 450.0)// cap integral
		integral_angle = 450.0;
	else if(integral_angle < -450.0)
		integral_angle = -450.0;
	double speed = ((k_pAngle*-error_angle)+(k_iAngle*-integral_angle)+(-(previous_angle-current_angle)*k_dAngle));
	if (speed > 1.0)
		speed = 1.0;
	else if (speed < -1.0)
		speed = -1.0;

	printf("speed: %f\n",speed);
	drive(speed,-speed);
	previous_angle = current_angle;

	return false;
}

bool Driving::AutoForwards(double goal)
{
	if (firstTime) {
	//	if (resetAuto()){
	//		firstTime = false;
	//	} else {
	//		return false;
	//	}
		previous_speed = 0.0;
		integral = 0;
		integral_angle = 0;
		error = 0;
		error_angle = 0;
		previous_angle = 0;

		intitalLeft = pos.left;
		intitalRight = pos.right;

		angleInitial = wharhs->GetAngle();
		updatePosition();
		firstTime = false;

	}


	SmartDashboard::PutNumber("angle for driving",wharhs->GetAngle());
	current_angle = wharhs->GetAngle() - angleInitial;

	double angle_factor = current_angle * k_ai;// * fabs( ( error_l ) / goal );

	current = (pos.right-intitalRight)*1.15*1.03;
	//printf("current %f\n", current);
	error = goal - current;

	/*speed = -error*k_p;
	integral += speed*k_i;
	if(integral > 0.2)
		integral = 0.2;
	if(integral < -0.2)
		integral = -0.2;
*/

	speed = -error *k_p + (integral)* k_i + k_d * (previous - error);

	//	drive(speed * 0.4, speed * 0.4);

	//speed is higher than 1. it equals max_speed;
	//speed is less than 1, it equals speed * mx_speed;


	if(fabs(current) > fabs(goal) || fabs(error) < TOLERANCE)
	{
		//printf("error: %f\n",error);
		printf("current after finished moving: %f\n",current);
		drive(0.0, 0.0);

		firstTime = true;
		return true;
	}




	if (speed > 0.75){
		speed = 0.75;
	}
	if (speed < -0.75){
		speed = -0.75;
	}

	//to stop it from wheely-ing at the beginning when it goes from 0 to 40amps. the torque jumps the robot. Helps it be gradual. More gradual at least.
	/*
	if(fabs(previous_speed - speed) > 0.6)
	{
		if (speed > 0){
			speed -= 0.6;
		}
		if (speed < 0){
			speed += 0.6;
		}
	}
	*/

	if(fabs(error/goal) > 0.90)
	{
		if (speed > fabs(1.0-error/goal + 0.25)){
			speed = fabs(1.0-error/goal+0.25);
		}
		if (speed < -fabs(1.0-error/goal+0.25)){
			speed = -fabs(1.0-error/goal+0.25);
		}
	}
/*
	if(fabs(error/goal) < 0.08)
	{
		printf("speed %f \n",speed);
		if (speed > fabs(error/goal)){
			speed = fabs(error/goal);
		}
		if (speed < -fabs(error/goal)){
			speed = -fabs(error/goal);
		}
	}
*/


	drive(speed + angle_factor, speed );

	previous = error;
	previous_speed = speed;
	return false;
}


bool Driving::AutoForwardsIndep(double goal)
{
	if (firstTime) {
		if (resetAuto()){
			firstTime = false;
		} else {
			return false;
		}
	}

	current_angle = wharhs->GetAngle() - angleInitial;

	printf("\nangle_error: %f \n", current_angle);

	current_l = pos.left*1.04;
	error_l = goal - current_l;
	double angle_factor = current_angle * k_ai;// * fabs( ( error_l ) / goal );
	//if(fabs(error_l) < 6.0)
	//	angle_factor = 0.0;


	speed_l = -error_l*k_p;
	integral_l += speed_l*k_i;
	if(integral_l > 0.2)
		integral_l = 0.2;
	if(integral_l < -0.2)
		integral_l = -0.2;

	current_r = pos.right*1.04;
	error_r = goal - current_r;
	speed_r = -error_r*k_p;
	integral_r += speed_r*k_i;
	if(integral_r > 0.2)
		integral_r = 0.2;
	if(integral_r < -0.2)
		integral_r = -0.2;

	//drive(speed * 0.4, speed * 0.4);
	//speed is higher than 1. it equals max_speed;
	//speed is less than 1, it equals speed * mx_speed;

	if(fabs(current_l) > fabs(goal) && fabs(current_r) > fabs(goal))//fabs(error) < TOLERANCE)
	{
		//printf("error: %f\n",(error_l + error_r)*0.5);
		drive(0.0, 0.0);
		firstTime = true;
		return true;
	}
	speed_r = speed_r*0.4+(integral_r);
	speed_l = speed_l*0.4+(integral_l);

	if (speed_l > 0.75){
		speed_l = 0.75;
	}
	else if (speed_l < -0.75){
		speed_l = -0.75;
	}

	if (speed_r > 0.75){
		speed_r = 0.75;
	}
	else if (speed_r < -0.75){
		speed_r = -0.75;
	}

	SmartDashboard::PutNumber("speed_l",speed_l + angle_factor);
	SmartDashboard::PutNumber("speed_r",speed_r);

	drive(speed_l + angle_factor, speed_r);

	return false;
}


bool Driving::turnNew ( double angle, double tolerance )
{
	if(firstTime)
	{
		integral = 0;
		integral_angle = 0;
		error = 0;
		error_angle = 0;
		previous_angle = 0;

		intitalLeft = pos.left;
		intitalRight = pos.right;

		angleInitial = wharhs->GetAngle();
		printf("inital %f\n",angleInitial);
		updatePosition();
		firstTime = false;
		k_pAngle = -0.0283*(fabs(angle)-30.0)+2.60;
		k_iAngle = -0.00001222*(fabs(angle)-30.0)+0.006;
	}


	double current_angle_gyro = wharhs->GetAngle() - angleInitial;

//	printf("/n it %f\t%f \n",angleInitial, current_angle_gyro);

	SmartDashboard::PutNumber("intial",angleInitial);

	double current_angle_enc = 1.45*signum(angle)*(((fabs(pos.left-intitalLeft)+fabs(pos.right-intitalRight)))/2.0) * 360.0 / (WHEELBASE * PI); //( fabs(pos.right - intitalRight) + fabs(pos.left - intitalLeft) ) * ( 180.0) / (WHEELBASE * PI);//   fabs(((fabs(pos.right - intitalRight) + fabs(pos.left - intitalLeft))/2.0) * (360.0) / ( WHEELBASE * PI)); //This gets the angle (deg) based on the encoders
//1.0367
//	printf(" gyro : %f\tenc: %f",current_angle_gyro, current_angle_enc);

	double alpha = 1.0; //This needs an experimental value
	double fused_angle = (current_angle_gyro*alpha) + (current_angle_enc*(1.0-alpha)); //The new current_angle, should it be instance?


	//From here down is me
	error_angle = angle - fused_angle;
//	printf("error_angle : %f \ n",error_angle);
	SmartDashboard::PutNumber("error angle!:", error_angle);
	if(fabs(error_angle) < tolerance || fabs(fused_angle) > fabs(angle-tolerance)){
		drive(0.0,0.0);
		firstTime = true;
		printf("final %f\n", wharhs->GetAngle());
		printf("final err %f\n", current_angle_gyro);
		return true;
	}

	error_angle /= 90.0;

	integral_angle += error_angle;
	if(integral_angle > 450.0)// cap integral
		integral_angle = 450.0;
	else if(integral_angle < -450.0)
		integral_angle = -450.0;

	double speed = ((k_pAngle*-error_angle)+(k_iAngle*-integral_angle)+(-(previous_angle-error_angle)*k_dAngle));
	if (speed > 1.0)
		speed = 1.0;
	else if (speed < -1.0)
		speed = -1.0;
//	printf("speed: %f\n",speed);
	drive(speed,-speed);
	previous_angle = error_angle; //Using fused instead of current

	return false;
}



bool Driving::turnUniversal ( double angleTo, double tolerance )
{
	if(firstTime)
	{
		integral = 0;
		integral_angle = 0;
		error = 0;
		error_angle = 0;
		previous_angle = 0;

		intitalLeft = pos.left;
		intitalRight = pos.right;

		angleInitial = wharhs->GetAngle();
		printf("inital %f\n",angleInitial);
		updatePosition();
		firstTime = false;
		angleDelta = angleTo - angleInitial;

		k_pAngle = -0.0283*(fabs(angleDelta)-30.0)+2.60;
		k_iAngle = -0.00001222*(fabs(angleDelta)-30.0)+0.006;
	}


	double current_angle_gyro = wharhs->GetAngle();// - angleInitial;

//	printf("/n it %f\t%f \n",angleInitial, current_angle_gyro);

	SmartDashboard::PutNumber("intial",angleInitial);

	double current_angle_enc = 1.45*signum(angleDelta)*(((fabs(pos.left-intitalLeft)+fabs(pos.right-intitalRight)))/2.0) * 360.0 / (WHEELBASE * PI); //( fabs(pos.right - intitalRight) + fabs(pos.left - intitalLeft) ) * ( 180.0) / (WHEELBASE * PI);//   fabs(((fabs(pos.right - intitalRight) + fabs(pos.left - intitalLeft))/2.0) * (360.0) / ( WHEELBASE * PI)); //This gets the angle (deg) based on the encoders
//1.0367
//	printf(" gyro : %f\tenc: %f",current_angle_gyro, current_angle_enc);

	double alpha = 1.0; //This needs an experimental value
	double fused_angle = (current_angle_gyro*alpha) + (current_angle_enc*(1.0-alpha)); //The new current_angle, should it be instance?


	//From here down is me
	error_angle = angleTo - fused_angle;
//	printf("error_angle : %f \ n",error_angle);
	SmartDashboard::PutNumber("error angle!:", error_angle);
	if(fabs(error_angle) < tolerance || fabs(fused_angle-angleInitial) > fabs(angleTo-tolerance)){
		drive(0.0,0.0);
		firstTime = true;
		printf("final %f\n", wharhs->GetAngle());
		printf("final err %f\n", current_angle_gyro);
		return true;
	}

	error_angle /= 90.0;

	integral_angle += error_angle;
	if(integral_angle > 450.0)// cap integral
		integral_angle = 450.0;
	else if(integral_angle < -450.0)
		integral_angle = -450.0;

	double speed = ((k_pAngle*-error_angle)+(k_iAngle*-integral_angle)+(-(previous_angle-error_angle)*k_dAngle));
	if (speed > 1.0)
		speed = 1.0;
	else if (speed < -1.0)
		speed = -1.0;
//	printf("speed: %f\n",speed);
	drive(speed,-speed);
	previous_angle = error_angle; //Using fused instead of current

	return false;
}

bool Driving::AutoForwardsUniversal(double goal, double angle)
{
	if (firstTime) {
	//	if (resetAuto()){
	//		firstTime = false;
	//	} else {
	//		return false;
	//	}
		previous_speed = 0.0;
		integral = 0;
		integral_angle = 0;
		error = 0;
		error_angle = 0;
		previous_angle = 0;

		intitalLeft = pos.left;
		intitalRight = pos.right;

		angleInitial = wharhs->GetAngle();
		updatePosition();
		firstTime = false;

	}


	SmartDashboard::PutNumber("angle for driving",wharhs->GetAngle());
	current_angle = wharhs->GetAngle() - angle;// - angleInitial;

	double angle_factor = current_angle * k_ai;// * fabs( ( error_l ) / goal );

	current = (pos.right-intitalRight)*1.15*1.03;
	//printf("current %f\n", current);
	error = goal - current;

	/*speed = -error*k_p;
	integral += speed*k_i;
	if(integral > 0.2)
		integral = 0.2;
	if(integral < -0.2)
		integral = -0.2;
*/

	speed = -error *k_p + (integral)* k_i + k_d * (previous - error);

	//	drive(speed * 0.4, speed * 0.4);

	//speed is higher than 1. it equals max_speed;
	//speed is less than 1, it equals speed * mx_speed;


	if(fabs(current) > fabs(goal) || fabs(error) < TOLERANCE)
	{
		//printf("error: %f\n",error);
		printf("current after finished moving: %f\n",current);
		drive(0.0, 0.0);

		firstTime = true;
		return true;
	}




	if (speed > 0.75){
		speed = 0.75;
	}
	if (speed < -0.75){
		speed = -0.75;
	}

	//to stop it from wheely-ing at the beginning when it goes from 0 to 40amps. the torque jumps the robot. Helps it be gradual. More gradual at least.
	/*
	if(fabs(previous_speed - speed) > 0.6)
	{
		if (speed > 0){
			speed -= 0.6;
		}
		if (speed < 0){
			speed += 0.6;
		}
	}
	*/

	if(fabs(error/goal) > 0.90)
	{
		if (speed > fabs(1.0-error/goal + 0.25)){
			speed = fabs(1.0-error/goal+0.25);
		}
		if (speed < -fabs(1.0-error/goal+0.25)){
			speed = -fabs(1.0-error/goal+0.25);
		}
	}
/*
	if(fabs(error/goal) < 0.08)
	{
		printf("speed %f \n",speed);
		if (speed > fabs(error/goal)){
			speed = fabs(error/goal);
		}
		if (speed < -fabs(error/goal)){
			speed = -fabs(error/goal);
		}
	}
*/


	drive(speed + angle_factor, speed );

	previous = error;
	previous_speed = speed;
	return false;
}


///PRECODITION:
// rplidar object is made.
// a call to
// lidar->begin(SerialPort::Port::kUSB1);
// lidar->lidar_thread();
// lidar->startScan(true,500);
// has been made.
// it will crash if not. an uncaught nll_ptr.
// IN MILIMETERS
bool Driving::LidarForwards(double goal, RPLidar * lidar)
{
	if (firstTime) {
		resetAuto();
		firstTime = false;
	}
	current = lidar->getRangeDistance(5,355,true);
	//printf("current %f\n", current);
	error = goal - current;
	speed = error*k_p;
	integral += speed*k_i;
	if(integral > 0.2)
		integral = 0.2;
	if(integral < -0.2)
		integral = -0.2;

	//drive(speed * 0.4, speed * 0.4);
	//speed is higher than 1. it equals max_speed;
	//speed is less than 1, it equals speed * mx_speed;

	if(fabs(current) > fabs(goal))//fabs(error) < TOLERANCE)
	{
		//printf("error: %f\n",error);
		drive(0.0, 0.0);
		firstTime = true;
		return true;
	}
	else if (speed > 1.0){
		drive(0.75, 0.75);
	}
	else if (speed < -1.0){
		drive(-0.75, -0.75);
	}
	else
	{
		drive((speed*0.4+(integral)), (speed*0.4+(integral)));
	}

	return false;
}


bool Driving::resetAuto(){
	resetPosition();
	resetEncoders();

	double encoders = left_encoder;
//	wharhs->Reset();
	integral = 0;
	integral_angle = 0;
	error = 0;
	error_angle = 0;
	previous_angle = 0;

	integral_r = 0;
	error_r = 0;
	integral_l = 0;
	error_l = 0;

	//reset global angle
	angleInitial = wharhs->GetAngle();
	//return encoders != prev_encoders;
	updatePosition();
	return true;
}

bool Driving::autoCube(Lift * lift){
	if (firstTime) {
		resetAuto();
		firstTime = false;
	}
	//k_pCube = SmartDashboard::GetNumber("K P cube",0.0);
	//k_iCube = SmartDashboard::GetNumber("K I cube",0.0);
	//k_dCube = SmartDashboard::GetNumber("K D cube",0.0);


	current = -SmartDashboard::GetNumber("cmxn2", 0.0);//may need to invert.
	if(fabs(current) < 0.07){ //Tolerance is 2.0. IS this an okay value? isn't it on [-1,1]?
		drive(0.0,0.0);
		firstTime = true;
		lift->runClaw(0.0);
		return true;
	}
	lift->runClaw(0.0);
	integral +=  current;
	double iMax = 0.3;
	if(integral_angle > iMax)// cap integral
		integral_angle = iMax;
	else if(integral_angle < -iMax)
		integral_angle = -iMax;
	double speed = ((k_pCube*-current)+(k_iCube*-integral)+(-(previous-current)*k_dCube));
	/*if (speed > 0.5)
		speed = 0.5;
	else if (speed < -0.5)
		speed = -0.5;*/

	double creep_bias = 0.0;
	//printf("oskar: %f\n",speed);

	drive(speed+creep_bias,-(speed+creep_bias));
	previous = current;

	return false;
}
