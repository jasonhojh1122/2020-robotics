#include <iostream>
#include <unistd.h>
#include <math.h>
#include "/usr/local/Aria/include/Aria.h"

const int D_T = 50; // ms
const double MAX_VEL = 350;
const double MAX_ROT_VEL = 10;
const double ACCEL = 100;
const double MIN_DIST = 50;

#define PI 3.141592653589793238463

double angleToRadians(double x) {
    return x * PI / 180.0;
}
double radiansToAngle(double x) {
    return x * 180.0 / PI;
}

double x, y, theta;

class Vec3 {
public:
	double x, y, z;
	Vec3(double x, double y, double z = 0) : x(x), y(y){
	}

	void cross(Vec3 v, double res[3]) {
		res[0] = y * v.z - z * v.y;
		res[1] = -(x * v.z - v.z * v.x);
		res[2] = x * v.y - y * v.x;
	}

    double dot(Vec3 v) {
        return x * v.x + y * v.y + z * v.z;
    }

	void rotate(double angle) {
		double xt, yt;
        double rAngle = angleToRadians(angle);
		xt = x * cos(rAngle) - y * sin(rAngle);
		yt = x * sin(rAngle) - y * cos(rAngle);
		x = xt;
		y = yt;
	}

    double calcRotation(Vec3 v) {
        double cosTh = dot(v) / (length() * v.length());
        return radiansToAngle(acos(cosTh));
    }

    double length() {
        return sqrt(x*x + y*y + z*z);
    }
};

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
	
	std::cin >> x >> y >> theta;
	
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

	robot.runAsync(false);

	while (true) {
		robot.lock();
		controller.rotVel = 0;
		robot.setRotVel(0);
		robot.unlock();

		Vec3 head = {0, 1, 0};
		Vec3 robotToTarget = {x - robot.getX(), y - robot.getY(), 0};
		head.rotate(robot.getTh());
		std::cout << head.calcRotation(robotToTarget) << '\n';

		if (controller.shouldStop()) {
			std::cout << "should stop\n";
			controller.vel *= 0.95;
			keyPressCallBack.updateRobotVel();
		}
		usleep(D_T);
	}

	Aria::shutdown();
	Aria::exit(0);
}