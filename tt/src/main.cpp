#include "main.h"
#include "api.h"
#include <math.h>
#include <string>
#include <bits/stdc++.h>
using namespace okapi;

pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::Imu in(20);
pros::Motor roller (16, pros::E_MOTOR_GEARSET_18, 0, pros::E_MOTOR_ENCODER_ROTATIONS);
pros::Motor sw (12, pros::E_MOTOR_GEARSET_06, 1, pros::E_MOTOR_ENCODER_ROTATIONS);
pros::Motor se (2, pros::E_MOTOR_GEARSET_06, 0, pros::E_MOTOR_ENCODER_ROTATIONS);
pros::Motor nw (11, pros::E_MOTOR_GEARSET_06, 1, pros::E_MOTOR_ENCODER_ROTATIONS);
pros::Motor ne (1, pros::E_MOTOR_GEARSET_06, 0, pros::E_MOTOR_ENCODER_ROTATIONS); 
pros::Motor intake(10, pros::E_MOTOR_GEARSET_06, 0, pros::E_MOTOR_ENCODER_ROTATIONS); 
pros::ADIEncoder ltw ('a', 'b', 0); 
pros::ADIEncoder rtw ('e', 'f', 1);
pros::ADIEncoder stw ('g', 'h', 0);
pros::ADIDigitalOut air ('d');
pros::ADIDigitalOut expansion('c');
//pros::Motor i2 (10, pros::E_MOTOR_GEARSET_18, 0, pros::E_MOTOR_ENCODER_ROTATIONS);
 
//flywheel
pros::Motor fa (17, pros::E_MOTOR_GEARSET_06, 1, pros::E_MOTOR_ENCODER_ROTATIONS); 
pros::Motor fb (13, pros::E_MOTOR_GEARSET_06, 0, pros::E_MOTOR_ENCODER_ROTATIONS);

//coloring
bool red;
pros::Optical optical_sensor(21);
  pros::c::optical_rgb_s_t rgb_value;

//flywheel pid
const double threshold = 500;
const double threshold2 = 500;
const double kV = 18.5185185;
const double kP = 10; 
const double kD = 150;
const double kV2 = 17;
const double kP2 = 30; 
const double kD2 = 150;
double afpe = 0, bfpe = 0;
double targetVelocity, targetVelocity2 =  250;
double test;

//odometry variables
double wd = 3.25; //wheel diameter
double twd = 2.75; //tracking wheel diamter
double sl=2.85, ss=3.02, sr=1.73; //distance from tracking wheels to center
double x,y,theta;
double prevL,prevR,prevS,L,R,S;
double h,i,a; //distance travelled, half of angle change, angle change
double h2; //same but for strafe
double r,r2; //cirle radius
double ox,oy,otheta; //version 2 of odon

double kp = 1900, ki = 0.5, kd = 10000; //1.9 for short 8000
//0.5 for 72 (i think?)
double ultotalerror,lltotalerror;
double ulpreverror = 0,llpreverror=0,llerrordiff = 0,ulerrordiff;
//added variables to make robot move straighter path
double distpreverror, disterrordiff, disttotalerror;

//drive straight
double prevFta;
double ktp = 0.1;

bool px = true;
void destodom(double target_x, double target_y){ //maybe add target orientation?
	double xcom, ycom, uld, lld, rta, fta, dist;
	xcom = target_x-x, ycom = target_y-y;
	if(xcom>0) fta = atan(ycom/xcom);
	else fta = atan(ycom/xcom)-M_PI;
	if(isnan(fta)||xcom==0) fta = M_PI/2;
		fta-=(fta-prevFta)*ktp;
	rta = fta-(M_PI/4-theta);
	dist = sqrt(xcom*xcom+ycom*ycom);
	//added manipulations to dist line (reversed the order of mulitplying by k constants):
	disttotalerror+=dist;
	disterrordiff = dist - distpreverror;
	distpreverror = dist;
	double mag = dist*kp + disterrordiff*kd + disttotalerror*ki;
	if(mag>12000) mag = 14000; //12000
	uld = cos(rta)*dist;
	lld = sin(rta)*dist;
	nw.move_voltage(cos(rta)*mag);
	se.move_voltage(cos(rta)*mag);
	ne.move_voltage(sin(rta)*mag);
	sw.move_voltage(sin(rta)*mag);
	prevFta = fta;
}
std::shared_ptr<OdomChassisController> chassis;
void odometry(){
	in.reset();
	in.tare();
	ltw.reset();
	stw.reset();
	rtw.reset();
	x = 0, y = 0, theta = 0;
	ox = 0, oy = 0, otheta = 0;
	prevL = prevR = prevS = 0;
	chassis =
 	ChassisControllerBuilder()
    .withMotors(11,-1,-2,12) 
    // green gearset, 4 inch wheel diameter, 11.5 inch wheel track
    .withDimensions(AbstractMotor::gearset::blue, {{4_in, 11.5_in}, imev5GreenTPR})
    // left encoder in ADI ports A & B, right encoder in ADI ports C & D (reversed)
    .withSensors(ADIEncoder{'A', 'B'}, ADIEncoder{'E', 'F', true}, ADIEncoder{'G','H',1})
    // specify the tracking wheels diameter (2.75 in), track (7 in), and TPR (360)
    .withOdometry({{2.75_in, 4.58_in,3_in,2.75_in}, quadEncoderTPR}, StateMode::CARTESIAN)
    .buildOdometry();
	double prevhe = 0;
	while(1){
		L = (ltw.get_value()-prevL)/360*twd*M_PI;
		R = (rtw.get_value()-prevR)/360*twd*M_PI;
		S = (stw.get_value()-prevS)/360*twd*M_PI;
		prevL = (double)(ltw.get_value());
		prevR = (double)(rtw.get_value());
		prevS = (double)(stw.get_value());
		a = (L-R)/(sl+sr); //added sr
		double headingdiff = in.get_heading()-prevhe;
		prevhe = in.get_heading();
		if(headingdiff<-50) headingdiff+=360;
		else if(headingdiff >50) headingdiff-=360;
		headingdiff*=M_PI/180;
		a = headingdiff;
		pros::lcd::print(5,"a%lf",a);
		if(a&&!isinf(a)&&!isnan(a)){
			r = R/a;
			i = a/2.0;
			h = ((r+sr)*sin(i))*2;
			r2 = S/a;
			h2 = ((r2+ss)*sin(i))*2;
		}
		else{
			h = R;
			i = 0;
			h2 = S;
		}
		double p = i+theta;
		y = y + h*cos(p) + h2*(-1*sin(p));
		x = x + h*sin(p) + h2*cos(p);
		if(!isinf(a)&&!isnan(a))theta+=a;
		//theta = theta%(2*M_PI); 
		//theta+=2*M_PI;
		//while(theta-2*M_PI>=0) theta-=2*M_PI;
		
		
		if(targetVelocity>-300){
			//double error = targetVelocity - ((fa.get_actual_velocity()+fb.get_actual_velocity())/2);
			//250 and 600 for shoot from mid
			double error = targetVelocity - fa.get_actual_velocity();
			double error2 = targetVelocity2 - fb.get_actual_velocity();
			double power,power2;
			double ad = error - afpe;
			double bd = error2 - bfpe;
			if(error > threshold){
				power = 12000;
			}
			else if(error < -threshold){
				power = 0;
			}
			else{
				power = kV * targetVelocity + kP*error; //kv supposed to be slope
			}
			if(error2 > threshold2){
				power2 = 12000;
			}
			else if(error2 < -threshold2){
				power2 = 0;
			}
			else{
				power2 = kV2 * targetVelocity2 + kP2*error2+50; //kv supposed to be slope
			}
			afpe = error;
			bfpe = error2;
			fb.move_voltage(power2);
			fa.move_voltage(power);
			pros::lcd::print(7,"avel%lf",fa.get_actual_velocity());
			pros::lcd::print(6,"bvel%lf",fb.get_actual_velocity());
		}
		
		std::string ff = chassis->OdomChassisController::getState().str(1_in,"_in",1_deg,"_deg");
		ff = ff.substr(10);
		pros::lcd::print(0,"%s",ff.c_str());
		pros::lcd::print(1,"%lf",test);
		std::string aa = "";
		double vall = stod(ff.substr(ff.find("x=")+2,ff.find("_in")));
		ox = vall;
		ff = ff.substr(ff.find("_in")+1);
		oy = stod(ff.substr(ff.find("y=")+2,ff.find("_in")));
		ff = ff.substr(ff.find("_in")+1);
		otheta = stod(ff.substr(ff.find("a=")+2,ff.find("_")));
		//USING RADIANS
		otheta = otheta * M_PI/180;
		pros::delay(10);
	}
}

void initialize() {
	red = true;
	expansion.pros::ADIDigitalOut::set_value(0);
	air.pros::ADIDigitalOut::set_value(0);
	pros::lcd::initialize();
	ne.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	nw.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	sw.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	se.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);	
	x = 0,y=0,theta=0;
	distpreverror = 0, disterrordiff = 0, disttotalerror = 0;
	in.reset();
	in.tare();
	ultotalerror=0,lltotalerror=0;
	pros::Task my_task(odometry);
}void disabled() {}void competition_initialize() {}

void getTo(double xx, double yy){
	disttotalerror = 0;
	double xcom, ycom, uld, lld, rta, fta, dist;
	xcom = xx-x, ycom = yy-y;
	if(xcom>0) fta = atan(ycom/xcom);
	else fta = atan(ycom/xcom)-M_PI;
	if(isnan(fta)||xcom==0) fta = M_PI/2;
	prevFta = fta;
	while(abs(xx-x)>0.1||abs(yy-y)>0.1){
		destodom(xx,yy);
		pros::delay(20);
	}
	ne.move_velocity(0);
	nw.move_velocity(0);
	se.move_velocity(0);
	sw.move_velocity(0);
}

void tr(double vel){
	ne = -vel;
	nw = vel;
	se = -vel;
	sw = vel;
}

double tkP = 8000; double tkI = 400, terror=0; double tkD = 800000;
double ppe = 0;
void turnTo(double aa){
	terror = 0;
	ppe = 0;
	while(theta<aa*M_PI/180-0.03||theta>aa*M_PI/180+0.03){
		double te = theta - aa*M_PI/180;
		double ec = te - ppe;
		ppe = te;
		terror = terror +te;
		pros::lcd::print(5,"%lf",te);
		nw.move_voltage((tkP*te+tkI*terror+ec*tkD)*-1);
		ne.move_voltage(tkP*te+tkI*terror+ec*tkD);
		sw.move_voltage((tkP*te+tkI*terror+ec*tkD)*-1);
		se.move_voltage(tkP*te+tkI*terror+ec*tkD);
		pros::delay(10);
	}
	nw.move_voltage(0);
		ne.move_voltage(0);
		sw.move_voltage(0);
		se.move_voltage(0);
}

void turnTo2(double aa){
	terror = 0;
	while(theta<aa*M_PI/180-0.03||theta>aa*M_PI/180+0.03){
		double te = theta - aa*M_PI/180;
		terror = terror +te;
		pros::lcd::print(5,"%lf",te);
		if(te>0){
		nw.move_velocity(-120);
		ne.move_velocity(120);
		sw.move_velocity(-120);
		se.move_velocity(120);
		}
		else{
			nw.move_velocity(120);
		ne.move_velocity(-120);
		sw.move_velocity(120);
		se.move_velocity(-120);
		}
		
		pros::delay(10);
	}
	nw.move_voltage(0);
		ne.move_voltage(0);
		sw.move_voltage(0);
		se.move_voltage(0);
}

void shoot(){
	air.set_value(1);
	pros::delay(500);
	air.set_value(0);
	pros::delay(500);
}

void roll(){
	roller.move_velocity(200);
	pros::delay(63);
	roller.move_velocity(0);
}

void winpoint(){
	air.set_value(0);
	//getTo(0,-1);
	nw.move_velocity(-100);
	se.move_velocity(-100);
	ne.move_velocity(-100);
	sw.move_velocity(-100);
	pros::delay(200);
	nw.move_velocity(0);
	se.move_velocity(0);
	ne.move_velocity(0);
	sw.move_velocity(0);
	roller.move_velocity(-200);
	pros::delay(200);
	roller.move_velocity(0);
	getTo(26,30); //10,14 24,28
	targetVelocity = 600;
	turnTo(-25);
	pros::delay(1000);
	air.set_value(1);
	pros::delay(800);
	air.set_value(0);
	pros::delay(800);
	air.set_value(1);
	pros::delay(180);
	air.set_value(0);
	pros::delay(800);
	air.set_value(1);
	pros::delay(180);
	air.set_value(0);
	pros::delay(800);
	targetVelocity = 0;
	turnTo(-35);
	getTo(60,118);
}

void URauton(){
	intake.move_velocity(600);
	targetVelocity = 600;
	getTo(-25,29);
	turnTo(-2);
	shoot();
	pros::delay(500);
	shoot();
	pros::delay(500);
	shoot();
	turnTo(-30);
	targetVelocity = 0;
	getTo(16,9);
}

void URauton2(){
	intake.move_velocity(600);	
	targetVelocity = 600;
	getTo(0,30);
	turnTo(6); //8 12 7
	getTo(2,30); // 5 35 4 34 3 33 3 32 3 31
	intake.move_velocity(-600);
	shoot();
	pros::delay(300);
	
	shoot();
	pros::delay(300);
	shoot();
	intake.move_velocity(-600);
	turnTo(-27);
	intake.move_velocity(0);
	
	nw.move_velocity(-600);
	se.move_velocity(-600);
	ne.move_velocity(-600);
	sw.move_velocity(-600);
	pros::delay(300);
	nw.move_velocity(0);
	se.move_velocity(0);
	ne.move_velocity(0);
	sw.move_velocity(0);
	getTo(23,2); //29 27 26
	//turnTo(12);
	roller.move_velocity(200);
	nw.move_velocity(-600);
	se.move_velocity(-600);
	ne.move_velocity(-600);
	sw.move_velocity(-600);
	pros::delay(390);
	nw.move_velocity(0);
	se.move_velocity(0);
	ne.move_velocity(0);
	sw.move_velocity(0);
	
	
	roll();
	nw.move_velocity(600);
	se.move_velocity(600);
	ne.move_velocity(600);
	sw.move_velocity(600);
	pros::delay(200);
	nw.move_velocity(0);
	se.move_velocity(0);
	ne.move_velocity(0);
	sw.move_velocity(0);
}

void awp(){
	expansion.set_value(0);
	targetVelocity2 = 380;
	targetVelocity = 380;

	nw.move_velocity(-600);
	se.move_velocity(-600);
	ne.move_velocity(-600);
	sw.move_velocity(-600);
	pros::delay(180);
	nw.move_velocity(0);
	se.move_velocity(0);
	ne.move_velocity(0);
	sw.move_velocity(0);
	roller.move_velocity(-60);
	pros::delay(770);
	roller.move_velocity(0);
	nw.move_velocity(600);
	se.move_velocity(600);
	ne.move_velocity(600);
	sw.move_velocity(600);
	pros::delay(170);
	nw.move_velocity(0);
	se.move_velocity(0);
	ne.move_velocity(0);
	sw.move_velocity(0);
	ki = 16;
	turnTo(-20);
	roller.move_velocity(-200);
	intake.move_velocity(-600);
	getTo(-1,11);
	//getTo(0,5);
	ki = 0.7;
	pros::delay(700);
	intake.move_velocity(0);
	roller.move_velocity(0);
	getTo(39,50);
	tkP = 45000;
	tkI = 200; //120
	turnTo2(-39); //38.5
	tkP=12000;
	targetVelocity2 = 320;
	targetVelocity = 320;
	intake.move_velocity(600);

			roller.move_velocity(200);
	pros::delay(170);
	intake.move_velocity(0);
			roller.move_velocity(0);
			targetVelocity2 = 410;
	targetVelocity = 410;
			pros::delay(1000);
	intake.move_velocity(600);
			roller.move_velocity(200);
	pros::delay(150);
	intake.move_velocity(0);
			roller.move_velocity(0);
			targetVelocity2 = 412;
	targetVelocity = 412;
			pros::delay(1000);

	intake.move_velocity(600);
			roller.move_velocity(200);
	pros::delay(800);
	intake.move_velocity(0);
			roller.move_velocity(0);
			tkI = 0;
			//turnTo(115);
			pros::delay(1000);
			getTo(100,100);
			turnTo(-90);
			nw.move_velocity(-600);
			roller.move_velocity(100);
	se.move_velocity(-600);
	ne.move_velocity(-600);
	sw.move_velocity(-600);
	pros::delay(220);
	nw.move_velocity(0);
	se.move_velocity(0);
	ne.move_velocity(0);
	sw.move_velocity(0);
	//roller.move_velocity(80);
	//pros::delay(150);
	pros::delay(150);
	roller.move_velocity(0);
	targetVelocity2 = 0;
	targetVelocity = 0;
}

void lrr(){
	if(red){
		int x = 0;
		
		while(!(optical_sensor.get_hue()>=0&&optical_sensor.get_hue()<40)&&!(optical_sensor.get_hue()>340&&optical_sensor.get_hue()<=360)){
			roller.move_velocity(50);
			pros::delay(5);
			if(master.get_digital(pros::E_CONTROLLER_DIGITAL_Y)){
			return;
		}
		px  = master.get_digital(pros::E_CONTROLLER_DIGITAL_X);
		}
		roller.move_velocity(0);
		
		while(!(optical_sensor.get_hue()>=200&&optical_sensor.get_hue()<320)){
			roller.move_velocity(30);
			pros::delay(5);
			if(master.get_digital(pros::E_CONTROLLER_DIGITAL_Y)){
			return;
		}
		px  = master.get_digital(pros::E_CONTROLLER_DIGITAL_X);
		}
		roller.move_velocity(0);
	}
	else{
		
		while(!(optical_sensor.get_hue()>=200&&optical_sensor.get_hue()<320)){
			roller.move_velocity(50);
			pros::delay(5);
			if(master.get_digital(pros::E_CONTROLLER_DIGITAL_Y)){
			return;
		}
		px  = master.get_digital(pros::E_CONTROLLER_DIGITAL_X);
		}
		roller.move_velocity(0);
		while(!(optical_sensor.get_hue()>=0&&optical_sensor.get_hue()<40)&&!(optical_sensor.get_hue()>340&&optical_sensor.get_hue()<=360)){
			roller.move_velocity(30);
			pros::delay(5);
			if(master.get_digital(pros::E_CONTROLLER_DIGITAL_Y)){
			return;
		}
		px  = master.get_digital(pros::E_CONTROLLER_DIGITAL_X);
		}
		roller.move_velocity(0);
	}
}

void rs(){
	expansion.set_value(0);
	targetVelocity2 = 370;
	targetVelocity = 370;
	roller.move_velocity(-200);
	intake.move_velocity(-600);
	ki = 2;
	kd = 10000;
	getTo(0,27);
	kd = 9000;
	turnTo(-90); 
	intake.move_velocity(0) ;
	roller.move_velocity(0);
	turnTo(19.8); 
	//getTo(0,5);
	ki = 1;
	//pros::delay(700);
	targetVelocity2 = 300;
	targetVelocity = 300;
	intake.move_velocity(600);

			roller.move_velocity(200);
		
	pros::delay(2000);
	intake.move_velocity(0);
			roller.move_velocity(0);
			getTo(30,-3);
			turnTo2(0);
	nw.move_velocity(-600);
	se.move_velocity(-600);
	ne.move_velocity(-600);
	sw.move_velocity(-600);
	pros::delay(550);
	nw.move_velocity(0);
	se.move_velocity(0);
	ne.move_velocity(0);
	sw.move_velocity(0);
	lrr();
	
	
}

void ls(){
	nw.move_velocity(-600);
	se.move_velocity(-600);
	ne.move_velocity(-600);
	sw.move_velocity(-600);
	pros::delay(180);
	nw.move_velocity(0);
	se.move_velocity(0);
	ne.move_velocity(0);
	sw.move_velocity(0);
	lrr();
	nw.move_velocity(600);
	se.move_velocity(600);
	ne.move_velocity(600);
	sw.move_velocity(600);
	pros::delay(70);
	nw.move_velocity(0);
	se.move_velocity(0);
	ne.move_velocity(0);
	sw.move_velocity(0);
}

void skills(){
	targetVelocity2 = 0;
	targetVelocity = 0;
	nw.move_velocity(-600);
	se.move_velocity(-600);
	ne.move_velocity(-600);
	sw.move_velocity(-600);
	pros::delay(180);
	nw.move_velocity(0);
	se.move_velocity(0);
	ne.move_velocity(0);
	sw.move_velocity(0);
	lrr();
	nw.move_velocity(600);
	se.move_velocity(600);
	ne.move_velocity(600);
	sw.move_velocity(600);
	pros::delay(170);
	nw.move_velocity(0);
	se.move_velocity(0);
	ne.move_velocity(0);
	sw.move_velocity(0);
	ki = 2;
	turnTo(-20);
	roller.move_velocity(-200);
	intake.move_velocity(-600);
	getTo(-18,22);
	roller.move_velocity(0);
	intake.move_velocity(0);
	turnTo(70);
	nw.move_velocity(-600);
	se.move_velocity(-600);
	ne.move_velocity(-600);
	sw.move_velocity(-600);
	pros::delay(700);
	nw.move_velocity(0);
	se.move_velocity(0);
	ne.move_velocity(0);
	sw.move_velocity(0);
	lrr();
	nw.move_velocity(600);
	se.move_velocity(600);
	ne.move_velocity(600);
	sw.move_velocity(600);
	pros::delay(1000);
	nw.move_velocity(0);
	se.move_velocity(0);
	ne.move_velocity(0);
	sw.move_velocity(0);
	turnTo2(40);
	pros::delay(4000);
	expansion.set_value(true);
	pros::delay(2000);
	expansion.set_value(false);
	pros::delay(1000);
	expansion.set_value(true);
}


void autonomous() {
	skills();
}


bool pu = false, pd = false,su=false,sd=0;
bool pa = false, pb = false;
bool cco = false; 

void opcontrol() {
	pros::Controller master(pros::E_CONTROLLER_MASTER);

	while (true){
		
	/*
		ne = master.get_analog(ANALOG_LEFT_Y) - master.get_analog(ANALOG_RIGHT_X) - master.get_analog(ANALOG_LEFT_X);
		nw = master.get_analog(ANALOG_LEFT_Y) + master.get_analog(ANALOG_RIGHT_X) + master.get_analog(ANALOG_LEFT_X);
		se = master.get_analog(ANALOG_LEFT_Y) - master.get_analog(ANALOG_RIGHT_X) + master.get_analog(ANALOG_LEFT_X);
		sw = master.get_analog(ANALOG_LEFT_Y) + master.get_analog(ANALOG_RIGHT_X) - master.get_analog(ANALOG_LEFT_X);

			pros::lcd::print(7,"avel%lf",fa.get_actual_velocity());
			pros::lcd::print(6,"bvel%lf",fb.get_actual_velocity());
*/
			
		double tilt = 45+in.get_heading()  ; //for upper right auton, -90
		if(master.get_analog(ANALOG_LEFT_X)==0){
			if(master.get_analog(ANALOG_LEFT_Y)<0) tilt+=180;
		}
		else if(master.get_analog(ANALOG_LEFT_X)>0){
			tilt-=(90-atan(master.get_analog(ANALOG_LEFT_Y)/master.get_analog(ANALOG_LEFT_X))*180/M_PI);
		}
		else{
			tilt-=(270-atan(master.get_analog(ANALOG_LEFT_Y)/master.get_analog(ANALOG_LEFT_X))*180/M_PI);
		}	
		double m = (master.get_analog(ANALOG_LEFT_Y))*(master.get_analog(ANALOG_LEFT_Y)) + (master.get_analog(ANALOG_LEFT_X))*(master.get_analog(ANALOG_LEFT_X));
		m = sqrt(m);
		m*=2;
		nw = cos(tilt*M_PI/180)*m + master.get_analog(ANALOG_RIGHT_X);
		se = cos(tilt*M_PI/180)*m - master.get_analog(ANALOG_RIGHT_X);
		sw = sin(tilt*M_PI/180)*m + master.get_analog(ANALOG_RIGHT_X);
		ne = sin(tilt*M_PI/180)*m - master.get_analog(ANALOG_RIGHT_X);

		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)&&master.get_digital(pros::E_CONTROLLER_DIGITAL_A)){
			in.reset();
			in.tare();
		}
		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_X)){
			lrr();
		}
		else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_B)){
			roller.move_velocity(-200);
		}
		else{
			roller.move_velocity(0);
		}
		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)==1){
			intake.move_velocity(600);
			//i2.move_velocity(200);
			roller.move_velocity(200);
		}
		else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)==1){
			intake.move_velocity(-600);
			//i2.move_velocity(-200);
			roller.move_velocity(-200);
		}
		else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_A)==1){
			roller.move_velocity(0);
			intake.move_velocity(600);
			//i2.move_velocity(0); 
		}
		else{
			intake.move_velocity(0);
			//i2.move_velocity(0);
			roller.move_velocity(0);
		}

		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)==1){
			targetVelocity = 500;
			targetVelocity2 = 300;

		}
		else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)==1){
			//targetVelocity = -380;
		    //fa.move_velocity(-350);
			//fb.move_velocity(-350);
			targetVelocity = 600;
			targetVelocity2 = 600;
		}
		else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)==1){
			targetVelocity = 500;
		}
		else{
			targetVelocity = 380;
			//fa.move_velocity(0);
			//fb.move_velocity(0);
			targetVelocity2 = 380;
		}
		/*
		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_A)==1&&pa==0){
			air.pros::ADIDigitalOut::set_value(1);
			pros::delay(180);
			air.pros::ADIDigitalOut::set_value(0);
		}*/
		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)==1&&pb==0&&master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)==1){
			if(cco){
				expansion.set_value(false);
				cco = false;
			}
			else{
				expansion.set_value(true);
				cco = 1;
			}
			
		}
		px  = master.get_digital(pros::E_CONTROLLER_DIGITAL_X);
		pa = master.get_digital(pros::E_CONTROLLER_DIGITAL_A);
		pb = master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT);
		pros::lcd::print(1,"x:%lf y:%lf",x,y);
		pros::lcd::print(2,"theta:%lf",(theta*180/M_PI));
		pros::lcd::print(3,"ltw:%d rtw:%d stw:%d", ltw.get_value(), rtw.get_value(), stw.get_value());
		pros::lcd::print(4,"co%lf",optical_sensor.get_hue());
		//pros::lcd::print(6,"tilt%lf",tilt);
		pros::delay(20);
	}
	expansion.pros::ADIDigitalOut::set_value(0);
	air.pros::ADIDigitalOut::set_value(0);
}

