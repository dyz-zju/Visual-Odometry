#ifndef CAMERA_H_
#define CAMERA_H_


#include <opencv2\opencv.hpp>

class Camera{

public:

	Camera(double width, double height, double fx, double fy, double cx, double cy,
		double k1 = 0.0, double k2 = 0.0, double p1 = 0.0, double p2 = 0.0, double k3 = 0.0);

	~Camera();

	inline int  width() const { return _width; }
	inline int height() const { return _height; }
	inline double  fx() const { return _fx; }
	inline double  fy() const { return _fy; }
	inline double  cx() const { return _cx; }
	inline double  cy() const { return _cy; }
	inline double  k1() const { return _d[0]; }
	inline double  k2() const { return _d[1]; }
	inline double  p1() const { return _d[2]; }
	inline double  p2() const { return _d[3]; }
	inline double  k3() const { return _d[4]; }

private:

	double _width, _height;
	double _fx, _fy;
	double _cx, _cy;
	bool _distortion;  // 是否有径向畸变
	double _d[5];
};
#endif // CAMERA_H_