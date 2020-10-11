#include <iostream>
#include <unistd.h>
#include <math.h>
#include "Aria.h"

const int D_T = 50; // ms
const double MAX_VEL = 350;
const double ERR_DIST = 50;
const double ERR_ANGLE = 5;

#define PI 3.141592653589793238463

double x, y, theta;

double myABS(double x) {
	if (x > 0) return x;
	else return -x;
}
double angleToRadians(double x) {
    return x * PI / 180.0;
}
double radiansToAngle(double x) {
    return x * 180.0 / PI;
}

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
		yt = x * sin(rAngle) + y * cos(rAngle);
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
	
	ArPose target = ArPose(0, 0, 0);

	bool onCoordniate;
	bool onCorrectDirection;
	bool onAngle;
	bool hasObstacle;
	bool goal;

	RobotController (ArRobot& robot, double distThreshold, 
						double targetX, double targetY, double targetTheta)
	: robot(robot), distThreshold(distThreshold){
		vel = 0;
		rotVel = 0;

		onAngle = false;
		onCoordniate = false;
		onCorrectDirection = false;
		hasObstacle = false;
		goal = false;

		target.setX(targetX);
		target.setY(targetY);
		target.setTh(targetTheta);

		mySonar = robot.findRangeDevice("sonar");
		if (mySonar == NULL) {
			std::cout << "No sonar device found.";
			exit(1);
		}
	}

	void turnLeft() { rotVel = 15; }
	void turnRight() { rotVel = -15; }
	void stopTurning() {rotVel = 0;}
	void moveForwardSlow() {vel = 100;}
	void moveForward() {vel = 200;}
	void stopMoving() {vel = 0;}
	void updateRobotVel() {
		robot.lock();
		robot.setVel(vel);
		robot.setRotVel(rotVel);
		robot.unlock();
	}

	double getAngleToTarget() {
		Vec3 head = Vec3(1, 0, 0);
		Vec3 robotToTarget = Vec3(target.getX() - robot.getX(), target.getY() - robot.getY(), 0);
		head.rotate(robot.getTh());
		return head.calcRotation(robotToTarget);
	}

	bool targetOnLeft() {
		Vec3 robotFacing = Vec3(1, 0, 0);
		Vec3 robotToTarget = Vec3(target.getX() - robot.getX(), target.getY() - robot.getY(), 0);
		
		robotFacing.rotate(robot.getTh());
		
		double res[3];
		robotFacing.cross(robotToTarget, res);
		if (res[2] < 0)
			return false;
		else
			return true;
	}

	void rotateToTarget() {
		double angleToTarget = getAngleToTarget();
		
		if (myABS(angleToTarget) < 0.5) {
			stopTurning();
			onCorrectDirection = true;
		}
		else {
			stopMoving();
			if (targetOnLeft()) turnLeft();
			else turnRight();
			onCorrectDirection = false;
		}
	}

	void avoidObstacle() {
		
		range = mySonar->currentReadingPolar(0, 359, &angle);

		if (range > distThreshold || angle > 90 || angle < -90) {
			hasObstacle = false;
			stopTurning();
			return;
		}
		
		hasObstacle = true;
		onCorrectDirection = false;
		moveForwardSlow();
		if (angle > 0)
			turnRight();
		else
			turnLeft();
	}

	void rotateToTargetAngle() {
		stopMoving();
		if (robot.getTh() - target.getTh() > ERR_ANGLE) {
			turnRight();
		}
		else if (robot.getTh() - target.getTh() < -ERR_ANGLE) {
			turnLeft();
		}
		else {
			onAngle = true;
			stopTurning();
		}
	}

	double distToTarget() {
		double a, b;
		a = robot.getX() - target.getX();
		b = robot.getY() - target.getY();
		return sqrt(a*a + b*b);
	}

	void checkOnCoordinate() {
		double dist = distToTarget();
		std::cout << robot.getX() << ' ' << robot.getY() << ' ' << target.getX() << ' ' << target.getY() << ' ' << dist << '\n';
		if (dist < ERR_DIST) {
			onCoordniate = true;
		}
	}

	void move() {
		
		avoidObstacle();
		if (hasObstacle) {
			updateRobotVel();
			return;
		}

		rotateToTarget();
		moveForward();

		if (distToTarget() < 800) {
			vel *= 0.5;
		}

		if (!onCoordniate) {
			checkOnCoordinate();
		}
		else if (onCoordniate && !onAngle) {
			rotateToTargetAngle();
		}
		else if (onCoordniate && onAngle) {
			stopTurning();
			stopMoving();
			goal = true;
		}
					
		updateRobotVel();
	}
};

int main(int argc, char **argv) {
	
	std::cout << "Coordinate System : \n"
	             "+x: the front of the robot's initial state\n"
	             "-x: the back  of the robot's initial state\n"
	             "+Y: the left  of the robot's initial state\n"
	             "+x: the right of the robot's initial state\n";

	std::cout << "Please input [x] [y] [theta], which are relative to the robot's initial state\n";

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

	RobotController controller = RobotController(robot, 800, x, y, theta);

	robot.attachKeyHandler(keyHandler);
	std::cout << "You may press escape to exit\n";

	robot.runAsync(false);

	while (!controller.goal) {
		controller.move();
		usleep(D_T);
	}

	Aria::shutdown();
	Aria::exit(0);
}