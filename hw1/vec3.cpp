#include <iostream>
#include <unistd.h>
#include <math.h>

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

int main() {
    Vec3 a = {0, 500, 0};
    a.rotate(0);
    std::cout << a.x << ' ' << a.y << ' ' << a.z;
}