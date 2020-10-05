#include <iostream>
#include <unistd.h>
#include "/usr/local/Aria/include/Aria.h"

const int D_T = 50; // ms
const double MAX_VEL = 350;
const double MAX_ROT_VEL = 10;
const double ACCEL = 100;

class RobotInfo {
public:
	RobotInfo () {
		vel = 0;
		rotVel = 0;
	}
	double vel;
	double rotVel;
};

class KeyPressCallBack {
public:
	KeyPressCallBack(ArRobot& robot, RobotInfo& robotInfo) : robot(robot), robotInfo(robotInfo) {
		updateRobotVel();
	}
    void key_w() {
		robotInfo.vel = checkVelBound(robotInfo.vel + ACCEL * D_T / 1000);
		robotInfo.rotVel = 0;
		updateRobotVel();
	}
	void key_s() {
		robotInfo.vel = checkVelBound(robotInfo.vel - ACCEL * D_T / 1000);
		robotInfo.rotVel = 0;
		updateRobotVel();
	}
	void key_a() {
		robotInfo.rotVel = MAX_ROT_VEL;
		updateRobotVel();
	}
	void key_d() {
		robotInfo.rotVel = -MAX_ROT_VEL;
		updateRobotVel();
	}

	ArRobot& robot;
	RobotInfo& robotInfo;

private:
	double checkVelBound(double vel) {
		if (vel > MAX_VEL)
			return MAX_VEL;
		else if (vel < -MAX_VEL)
			return -MAX_VEL;
		else
			return vel;
	}
	double checkRotVelBound(double rotVel) {
		if (rotVel > MAX_ROT_VEL)
			return MAX_ROT_VEL;
		else if (rotVel < -MAX_ROT_VEL)
			return -MAX_ROT_VEL;
		else
			return rotVel;
	}
	void updateRobotVel() {
		robot.lock();
		robot.setVel(robotInfo.vel);
		robot.setRotVel(robotInfo.rotVel);
		robot.unlock();
	}
};

int main(int argc, char **argv) {
	RobotInfo robotInfo = {};
	ArRobot robot;
	ArSonarDevice sonar;
	
	robot.addRangeDevice(&sonar);

	Aria::init();
	
	ArSimpleConnector connector(&argc,argv);

	if (!connector.connectRobot(&robot)){
		printf("Could not connect to robot... exiting\n");
		Aria::shutdown();
		Aria::exit(1);
	}

	robot.comInt(ArCommands::ENABLE, 1);
	robot.comInt(ArCommands::SONAR, 1);

	robot.setAbsoluteMaxTransVel(MAX_VEL);
	robot.setAbsoluteMaxRotVel(MAX_VEL);
	
	ArKeyHandler *keyHandler = Aria::getKeyHandler();
    if (keyHandler == NULL) {
        keyHandler = new ArKeyHandler;
        Aria::setKeyHandler(keyHandler);
    }

	KeyPressCallBack keyPressCallBack = {robot, robotInfo};
    ArFunctorC<KeyPressCallBack> key_w_pressed(keyPressCallBack, &KeyPressCallBack::key_w);
	ArFunctorC<KeyPressCallBack> key_s_pressed(keyPressCallBack, &KeyPressCallBack::key_s);
	ArFunctorC<KeyPressCallBack> key_a_pressed(keyPressCallBack, &KeyPressCallBack::key_a);
	ArFunctorC<KeyPressCallBack> key_d_pressed(keyPressCallBack, &KeyPressCallBack::key_d);

    keyHandler->addKeyHandler('w', &key_w_pressed);
	keyHandler->addKeyHandler('s', &key_s_pressed);
	keyHandler->addKeyHandler('a', &key_a_pressed);
	keyHandler->addKeyHandler('d', &key_d_pressed);

	robot.attachKeyHandler(keyHandler);
	printf("You may press escape to exit\n");

	robot.runAsync(false);

	while (true) {
		robot.lock();
		robotInfo.rotVel = 0;
		robot.setRotVel(0);
		robot.unlock();
		usleep(D_T);
	}

	Aria::shutdown();
	Aria::exit(0);
}