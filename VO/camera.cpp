#include "camera.h"

Camera::Camera(double width, double height,
	double fx, double fy,
	double cx, double cy,
	double k1, double k2, double p1, double p2, double k3):
	_width(width), _height(height), _fx(fx), _fy(fy), _cx(cx), _cy(cy),
	_distortion(fabs(k1) > 0.0000001)
{
	_d[0] = k1; 
	_d[1] = k2;
	_d[2] = p1;
	_d[3] = p2;
	_d[4] = k3;
}

Camera::~Camera() {}