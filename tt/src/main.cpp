#include "main.h"
#include "api.h"
#include <math.h>
#include <string>
using namespace okapi;

pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::Imu in(20);
pros::Motor roller (15, pros::E_MOTOR_GEARSET_18, 0, pros::E_MOTOR_ENCODER_ROTATIONS);
pros::Motor sw (12, pros::E_MOTOR_GEARSET_06, 1, pros::E_MOTOR_ENCODER_ROTATIONS);
pros::Motor se (2, pros::E_MOTOR_GEARSET_06, 0, pros::E_MOTOR_ENCODER_ROTATIONS);
pros::Motor nw (11, pros::E_MOTOR_GEARSET_06, 1, pros::E_MOTOR_ENCODER_ROTATIONS);
pros::Motor ne (1, pros::E_MOTOR_GEARSET_06, 0, pros::E_MOTOR_ENCODER_ROTATIONS); 
pros::Motor intake(16, pros::E_MOTOR_GEARSET_06, 0, pros::E_MOTOR_ENCODER_ROTATIONS); 
pros::ADIEncoder ltw ('a', 'b', 0); 
pros::ADIEncoder rtw ('e', 'f', 1);
pros::ADIEncoder stw ('g', 'h', 0);
pros::ADIDigitalOut air ('c');
pros::ADIDigitalOut expansion('d');
pros::Motor i2 (10, pros::E_MOTOR_GEARSET_18, 0, pros::E_MOTOR_ENCODER_ROTATIONS);
 
//flywheel
pros::Motor fa (17, pros::E_MOTOR_GEARSET_06, 1, pros::E_MOTOR_ENCODER_ROTATIONS); 
pros::Motor fb (13, pros::E_MOTOR_GEARSET_06, 0, pros::E_MOTOR_ENCODER_ROTATIONS);

//flywheel pid
const double threshold = 50;
const double kV = 20;
const double kP = 40; 
double targetVelocity, targetVelocity2 =  250;

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

double kp = 1100, ki = 0.5, kd = 260; //1.9 for short
//0.5 for 72 (i think?)
double ultotalerror,lltotalerror;
double ulpreverror = 0,llpreverror=0,llerrordiff = 0,ulerrordiff;
//added variables to make robot move straighter path
double distpreverror, disterrordiff, disttotalerror;

//drive straight
double prevFta;
double ktp = 0.004;
void destodom(double target_x, double target_y){ //maybe add target orientation?
	double xcom, ycom, uld, lld, rta, fta, dist;
	xcom = target_x-ox, ycom = target_y-oy;
	if(xcom>0) fta = atan(ycom/xcom);
	else fta = atan(ycom/xcom)-M_PI;
	if(isnan(fta)||xcom==0) fta = M_PI/2;
		fta-=(fta-prevFta)*ktp;
	rta = fta-(M_PI/4-otheta);
	dist = sqrt(xcom*xcom+ycom*ycom);
	//added manipulations to dist line (reversed the order of mulitplying by k constants):
	disttotalerror+=dist;
	disterrordiff = dist - distpreverror;
	distpreverror = dist;
	double mag = dist*kp + disterrordiff*kd + disttotalerror*ki;
	if(mag>12000) mag = 12000;
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
			if(error > threshold){
				power = 12000;
			}
			else if(error < -threshold){
				power = 0;
			}
			else{
				power = kV * targetVelocity + kP*error; //kv supposed to be slope
			}
			if(error2 > threshold){
				power2 = 12000;
			}
			else if(error2 < -threshold){
				power2 = 0;
			}
			else{
				power2 = kV * targetVelocity2 + kP*error2; //kv supposed to be slope
			}
			fb.move_voltage(power2);
			fa.move_voltage(power);
		}
		std::string ff = chassis->OdomChassisController::getState().str(1_in,"_in",1_deg,"_deg");
		ff = ff.substr(10);
		pros::lcd::print(0,"%s",ff.c_str());
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
	xcom = xx-ox, ycom = yy-oy;
	if(xcom>0) fta = atan(ycom/xcom);
	else fta = atan(ycom/xcom)-M_PI;
	if(isnan(fta)||xcom==0) fta = M_PI/2;
	prevFta = fta;
	while(abs(xx-ox)>0.1||abs(yy-oy)>0.1){
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

double tkP = 10000;
void turnTo(double aa){
	while(otheta<aa*M_PI/180-0.03||otheta>aa*M_PI/180+0.03){
		double te = otheta - aa*M_PI/180;
		pros::lcd::print(5,"%lf",te);
		nw.move_voltage(tkP*te*-1);
		ne.move_voltage(tkP*te);
		sw.move_voltage(tkP*te*-1);
		se.move_voltage(tkP*te);
		pros::delay(10);
	}
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

void autonomous() {
	expansion.set_value(0);

	targetVelocity2 = 400;
	targetVelocity = 400;
	pros::delay(3000);
	intake.move_velocity(600);
			i2.move_velocity(200);
			roller.move_velocity(200);
	pros::delay(160);
	intake.move_velocity(0);
			i2.move_velocity(0);
			roller.move_velocity(0);
			
			
			pros::delay(2000);
	targetVelocity2 = 0;
	targetVelocity = 0;
/*
	nw.move_velocity(-600);
	se.move_velocity(-600);
	ne.move_velocity(-600);
	sw.move_velocity(-600);
	pros::delay(500);
	nw.move_velocity(0);
	se.move_velocity(0);
	ne.move_velocity(0);
	sw.move_velocity(0);
	roller.move_velocity(80);
	pros::delay(200);
	roller.move_velocity(0);
	*/

	//ki = 1.9;
	//getTo(0,5);
	//turnTo(-90);
}


bool pu = false, pd = false,su=false,sd=0;
bool pa = false, pb = false;
bool cco = false;

void opcontrol() {
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	while (true){
	
		ne = master.get_analog(ANALOG_LEFT_Y) - master.get_analog(ANALOG_RIGHT_X) - master.get_analog(ANALOG_LEFT_X);
		nw = master.get_analog(ANALOG_LEFT_Y) + master.get_analog(ANALOG_RIGHT_X) + master.get_analog(ANALOG_LEFT_X);
		se = master.get_analog(ANALOG_LEFT_Y) - master.get_analog(ANALOG_RIGHT_X) + master.get_analog(ANALOG_LEFT_X);
		sw = master.get_analog(ANALOG_LEFT_Y) + master.get_analog(ANALOG_RIGHT_X) - master.get_analog(ANALOG_LEFT_X);

/*
		double tilt = 45+in.get_heading() ; //for upper right auton, -90
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
		m*=2;
		m = sqrt(m);
		nw = cos(tilt*M_PI/180)*m + master.get_analog(ANALOG_RIGHT_X);
		se = cos(tilt*M_PI/180)*m - master.get_analog(ANALOG_RIGHT_X);
		sw = sin(tilt*M_PI/180)*m + master.get_analog(ANALOG_RIGHT_X);
		ne = sin(tilt*M_PI/180)*m - master.get_analog(ANALOG_RIGHT_X);*/
		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)&&master.get_digital(pros::E_CONTROLLER_DIGITAL_A)){
			in.reset();
			in.tare();
		}
		
		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_X)){
			roller.move_velocity(200);
		}
		else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_B)){
			roller.move_velocity(-200);
		}
		else{
			roller.move_velocity(0);
		}
		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)==1){
			intake.move_velocity(600);
			i2.move_velocity(200);
			roller.move_velocity(200);
		}
		else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)==1){
			intake.move_velocity(-600);
			i2.move_velocity(-200);
			roller.move_velocity(-200);
		}
		else{
			intake.move_velocity(0);
			i2.move_velocity(0);
			roller.move_velocity(0);
		}

		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)==1){
			targetVelocity = 600;

		}
		else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)==1){
			//targetVelocity = -380;
		    //fa.move_velocity(-350);
			//fb.move_velocity(-350);
			targetVelocity = 400;
		}
		else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)==1){
			targetVelocity = 500;
		}
		else{
			targetVelocity = 0;
			//fa.move_velocity(0);
			//fb.move_velocity(0);
		}
		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_A)==1&&pa==0){
			air.pros::ADIDigitalOut::set_value(1);
			pros::delay(180);
			air.pros::ADIDigitalOut::set_value(0);
		}
		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)==1&&pb==0){
			if(cco){
				expansion.set_value(false);
				cco = false;
			}
			else{
				expansion.set_value(true);
				cco = 1;
			}
			
		}
		pa = master.get_digital(pros::E_CONTROLLER_DIGITAL_A);
		pb = master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT);
		pros::lcd::print(1,"x:%lf y:%lf",x,y);
		pros::lcd::print(2,"theta:%lf",(theta*180/M_PI));
		pros::lcd::print(3,"ltw:%d rtw:%d stw:%d", ltw.get_value(), rtw.get_value(), stw.get_value());
		pros::lcd::print(4,"imu%lf",in.get_heading());
		pros::delay(20);
	}
	expansion.pros::ADIDigitalOut::set_value(0);
	air.pros::ADIDigitalOut::set_value(0);
}

