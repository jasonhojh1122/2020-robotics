#include <iostream>
#include <unistd.h>
#include "/usr/local/Aria/include/Aria.h"

const int D_T = 50; // ms
const double MAX_VEL = 350;
const double MAX_ROT_VEL = 10;
const double ACCEL = 100;
const double MIN_DIST = 50;

class RobotController {
public:

	double vel;
	double rotVel;

	ArRobot& robot;
	ArRangeDevice* mySonar;
	double distThreshold;
	double range;
	double angle;

	RobotController (ArRobot& robot, double distThreshold)
	: robot(robot), distThreshold(distThreshold) {
		vel = 0;
		rotVel = 0;
		mySonar = robot.findRangeDevice("sonar");

		ArPose pose = {0, 0, 0};
		robot.moveTo(pose);

		if (mySonar == NULL) {
			std::cout << "No sonar device found.";
			exit(1);
		}
	}

	void readSurrondingDist() {
		range = mySonar->currentReadingPolar(0, 359, &angle);
	}

	bool obstacleInSameDirection() {
		if (angle < 60 && angle > -60 && vel > 0) {
			return true;
		}
		else if ((angle > 150 || angle < -150) && vel < 0){
			return true;
		}
		else {
			return false;
		}
	}

	bool shouldStop() {
		readSurrondingDist();
		if (range < distThreshold && obstacleInSameDirection())
			return true;
		else
			return false;
	}
};

class KeyPressCallBack {
public:
	KeyPressCallBack(ArRobot& robot, RobotController& controller)
	: robot(robot), controller(controller) {
		updateRobotVel();
	}
    void key_w() {
		controller.vel = checkVelBound(controller.vel + ACCEL * D_T / 1000);
		controller.rotVel = 0;
		updateRobotVel();
	}
	void key_s() {
		controller.vel = checkVelBound(controller.vel - ACCEL * D_T / 1000);
		controller.rotVel = 0;
		updateRobotVel();
	}
	void key_a() {
		controller.rotVel = MAX_ROT_VEL;
		updateRobotVel();
	}
	void key_d() {
		controller.rotVel = -MAX_ROT_VEL;
		updateRobotVel();
	}

	ArRobot& robot;
	RobotController& controller;

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
		robot.setVel(controller.vel);
		robot.setRotVel(controller.rotVel);
		robot.unlock();
	}
};

int main(int argc, char **argv) {
	ArRobot robot;
	ArSonarDevice sonar;
	
	robot.addRangeDevice(&sonar);

	Aria::init();
	
	ArSimpleConnector connector(&argc,argv);

	if (!connector.connectRobot(&robot)){
		std::cout << "Could not connect to robot... exiting\n";
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

	RobotController controller = {robot, 600};

	KeyPressCallBack keyPressCallBack = {robot, controller};
    ArFunctorC<KeyPressCallBack> key_w_pressed(keyPressCallBack, &KeyPressCallBack::key_w);
	ArFunctorC<KeyPressCallBack> key_s_pressed(keyPressCallBack, &KeyPressCallBack::key_s);
	ArFunctorC<KeyPressCallBack> key_a_pressed(keyPressCallBack, &KeyPressCallBack::key_a);
	ArFunctorC<KeyPressCallBack> key_d_pressed(keyPressCallBack, &KeyPressCallBack::key_d);

    keyHandler->addKeyHandler('w', &key_w_pressed);
	keyHandler->addKeyHandler('s', &key_s_pressed);
	keyHandler->addKeyHandler('a', &key_a_pressed);
	keyHandler->addKeyHandler('d', &key_d_pressed);

	robot.attachKeyHandler(keyHandler);
	std::cout << "You may press escape to exit\n";
	std::cout << "You may press w to go forward\n";
	std::cout << "You may press s to go backward\n";
	std::cout << "You may press a to go left\n";
	std::cout << "You may press d to go right\n";

	robot.runAsync(false);

	while (true) {
		robot.lock();
		controller.rotVel = 0;
		robot.setRotVel(0);
		robot.unlock();
		if (controller.shouldStop()) {
			controller.vel *= 0.95;
			keyPressCallBack.updateRobotVel();
		}
		usleep(D_T);
	}

	Aria::shutdown();
	Aria::exit(0);
}